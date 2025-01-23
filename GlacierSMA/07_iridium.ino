// Configure RockBLOCK 9603
void configureIridium()
{
  #if !NO_TRANSMIT
    modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power (USB power: IridiumSBD::USB_POWER_PROFILE)
    modem.adjustSendReceiveTimeout(iridiumTimeout);           // Timeout for Iridium send/receive commands (default = 300 s)
    modem.adjustStartupTimeout(iridiumTimeout / 2);           // Timeout for Iridium startup (default = 240 s)
    DEBUG_PRINTLN("Info - Iridium modem ready");
  #endif
}

// Write data from structure to transmit buffer
void writeBuffer()
{
  iterationCounter++; // Increment iteration counter
  transmitCounter++; // Increment data transmission counter
  moSbdMessage.iterationCounter = iterationCounter; // Write message counter data to union

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(moSbdBuffer + (sizeof(moSbdMessage) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)), moSbdMessage.bytes, sizeof(moSbdMessage));

  // Print MO-SBD union/structure
  printMoSbd();
  //printMoSbdHex();
  //printMoSbdBuffer();

  // Clear MO-SBD message union/structure
  memset(&moSbdMessage, 0x00, sizeof(moSbdMessage));
}

// Attempt to transmit data via RockBLOCK 9603
void transmitData()
{
  #if NO_TRANSMIT
    DEBUG_PRINTLN("Info - Satellite messages inhibited (#NO_TRANSMIT)");
    return;
  #endif

  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if data transmission interval has been reached
  if ((transmitCounter == transmitInterval) || firstTimeFlag)
  {
    // Enable power to the RockBLOCK 9603
    enable5V();

    // Open the Iridium serial port
    IRIDIUM_PORT.begin(19200);

    // Assign pins for SERCOM functionality for new Serial2 instance
    pinPeripheral(PIN_IRIDIUM_TX, PIO_SERCOM);
    pinPeripheral(PIN_IRIDIUM_RX, PIO_SERCOM);

    // Wake up the RockBLOCK 9603 and begin communications
    DEBUG_PRINTLN("Info - Starting iridium modem...");
    petDog(); // The following might take a while, so best reset the WDT here

    int returnCode = modem.begin();

    if (returnCode != ISBD_SUCCESS)
    {
      online.iridium = false;
      if (returnCode == ISBD_NO_MODEM_DETECTED) {
        DEBUG_PRINTLN("Warning - No modem detected! Please check wiring.");
      }
      else {
        DEBUG_PRINT("Warning - Modem begin failed with error "); DEBUG_PRINTLN(returnCode);
      }
    }
    else
    {
      online.iridium = true;

      // Calculate SBD message buffer sizes
      moSbdBufferSize = sizeof(moSbdMessage) * (transmitCounter + (retransmitCounter * transmitInterval));
      mtSbdBufferSize = sizeof(mtSbdBuffer);
      memset(mtSbdBuffer, 0x00, sizeof(mtSbdBuffer)); // Clear MT-SBD buffer

      DEBUG_PRINTLN("Info - Attempting to transmit message...");
      petDog(); // The following might take a while, so best reset the WDT here

      // Transmit and receieve SBD message data in binary format
      returnCode = modem.sendReceiveSBDBinary(moSbdBuffer, moSbdBufferSize, mtSbdBuffer, mtSbdBufferSize);

      // Check if transmission was successful
      if (returnCode == ISBD_SUCCESS)
      {
        DEBUG_PRINTLN("Info - MO-SBD message transmission successful!");
        blinkLed(PIN_LED_GREEN, 10, 500);

        failureCounter = 0; // Clear failed transmission counter
        retransmitCounter = 0; // Clear message retransmit counter
        memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer)); // Clear MO-SBD message buffer

        // Check if a Mobile Terminated (MT) SBD message was received
        // If no message is available, mtSbdBufferSize = 0
        if (mtSbdBufferSize > 0)
        {
          DEBUG_PRINT("Info - MT-SBD message received. Size: ");
          DEBUG_PRINT(mtSbdBufferSize); DEBUG_PRINTLN(" bytes.");
          printMtSbdBuffer(); // Print MT-SBD message in hexadecimal

          // Check if MT-SBD message is the correct size
          if (mtSbdBufferSize >= sizeof(mtSbdMessage))
          {
            DEBUG_PRINTLN("Info - MT-SBD message correct size.");

            // Write incoming MT-SBD message to union/structure
            memcpy(mtSbdMessage.bytes, mtSbdBuffer, sizeof(mtSbdMessage));

            // Print MT-SBD message
            printMtSbd(); // Print MT-SBD message stored in union/structure

            // Check if MT-SBD message data is valid and update variables
            if ((mtSbdMessage.sampleInterval    >= 1  &&  mtSbdMessage.sampleInterval   <= 60)  &&
                (mtSbdMessage.averageInterval   >= 1  &&  mtSbdMessage.averageInterval  <= 240)  &&
                (mtSbdMessage.transmitInterval  >= 1  &&  mtSbdMessage.transmitInterval <= 24)  &&
                (mtSbdMessage.retransmitLimit   >= 0  &&  mtSbdMessage.retransmitLimit  <= 5)  &&
                (mtSbdMessage.batteryCutoff     >= 0  &&  mtSbdMessage.batteryCutoff    <= 12)  &&
                (mtSbdMessage.resetFlag         == 0  ||  mtSbdMessage.resetFlag        == 255))
            {
              DEBUG_PRINTLN("Info - All received values within accepted ranges.");

              sampleInterval    = mtSbdMessage.sampleInterval;    // Update alarm interval
              averageInterval   = mtSbdMessage.averageInterval;   // Update sample average interval
              transmitInterval  = mtSbdMessage.transmitInterval;  // Update transmit interval
              retransmitLimit   = mtSbdMessage.retransmitLimit;   // Update retransmit limit
              batteryCutoff     = mtSbdMessage.batteryCutoff;     // Update battery cutoff voltage
              resetFlag         = mtSbdMessage.resetFlag;         // Update force reset flag
            }
            else
            {
              DEBUG_PRINT("Warning - Received values exceed accepted range!");
            }
          }
          else
          {
            DEBUG_PRINTLN("Warning - MT-SBD message incorrect size!");
          }
        }
      }
      else
      {
        DEBUG_PRINT("Warning - Transmission failed with error code ");
        DEBUG_PRINTLN(returnCode);
        blinkLed(PIN_LED_RED, 10, 500);
      }
    }

    petDog();

    // Store return status code
    transmitStatus = returnCode;
    DEBUG_PRINT("transmitStatus: "); DEBUG_PRINTLN(transmitStatus);
    moSbdMessage.transmitStatus = transmitStatus;

    // Store message in transmit buffer if transmission or modem begin fails
    if (returnCode != ISBD_SUCCESS)
    {
      retransmitCounter++;
      failureCounter++;

      // Reset counter if reattempt limit is exceeded
      if (retransmitCounter > retransmitLimit)
      {
        retransmitCounter = 0;
        memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer)); // Clear transmit buffer
      }
    }

    // Clear transmit buffer if program running for the first time
    if (firstTimeFlag)
    {
      retransmitCounter = 0;
      memset(moSbdBuffer, 0x00, sizeof(moSbdBuffer)); // Clear moSbdBuffer array
    }

    // Put modem to sleep
    if (returnCode != ISBD_NO_MODEM_DETECTED) {
      DEBUG_PRINTLN("Info - Putting modem to sleep...");
      returnCode = modem.sleep();
      if (returnCode != ISBD_SUCCESS)
      {
        DEBUG_PRINT("Warning - Sleep failed error "); DEBUG_PRINTLN(returnCode);
      }
    }

    // Close the Iridium serial port
    IRIDIUM_PORT.end();

    // Disable power to the RockBLOCK 9603
    disable5V();

    // Reset transmit counter
    transmitCounter = 0;

    // Stop the loop timer
    timer.iridium = millis() - loopStartTime;

    // Write duration of last transmission to union
    moSbdMessage.transmitDuration = timer.iridium / 1000;

    // Check if reset flag was transmitted
    if (resetFlag)
    {
      forceReset();
    }
  }
}

// Non-blocking RockBLOCK callback function can be called during transmit or GNSS signal acquisition
bool ISBDCallback()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000)
  {
    previousMillis = currentMillis;
    petDog(); // Reset the Watchdog Timer
    DEBUG_PRINT('.');
    digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LED_GREEN)); // Blink LED
  }
  return true;
}

#if DEBUG_IRIDIUM
// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  DEBUG_WRITE(c);
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  DEBUG_WRITE(c);
}
#endif
