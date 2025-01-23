// Read battery voltage from voltage divider
void readBattery()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Measure external battery voltage across 10/1 MΩ resistor divider (1/10 divider)
  (void)analogRead(PIN_VBAT);
  voltage = analogRead(PIN_VBAT);
  voltage *=  ((10000000.0 + 1000000.0) / 1000000.0); // Multiply back 1 MOhm / (10 MOhm + 1 MOhm)
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 4096;  // Convert to voltage

  // Measure LiPo battery voltage across 100 kΩ/100 kΩ onboard resistor divider (1/2 divider)
  //float voltage = analogRead(A7);
  //voltage = voltage * 3.3 * 2 / 4096.0;

  // Write data to union
  moSbdMessage.voltage = voltage * 100;

  // Add to statistics object
  batteryStats.add(voltage);

  // Stop loop timer
  timer.readBattery = millis() - loopStartTime;
}

// Disable serial port
void disableSerial()
{
#if DEBUG
  SERIAL_PORT.end(); // Close serial port
  USBDevice.detach(); // Safely detach USB prior to sleeping
#endif
}

// Enable serial port
void enableSerial()
{
#if DEBUG
  USBDevice.attach(); // Re-attach USB
  SERIAL_PORT.begin(115200);
  blinkLed(PIN_LED_RED, 4, 250); // Blink LED
  myDelay(1000); // Non-blocking delay to allow user to open Serial Monitor
#endif
}

// Enable power to GNSS
void enableGnssPower()
{
  digitalWrite(PIN_GNSS_EN, LOW);
  myDelay(1000);
}

// Disable power to GNSS
void disableGnssPower()
{
  digitalWrite(PIN_GNSS_EN, HIGH);
}

// Enable 5 V power
void enable5V()
{
  digitalWrite(PIN_5V_EN, HIGH);
  myDelay(500);
}

// Disable 5V power
void disable5V()
{
  digitalWrite(PIN_5V_EN, LOW);
}

// Enable 12 V power
void enable12V()
{
  digitalWrite(PIN_12V_EN, HIGH);
  myDelay(500);
}

// Disable 12 V power
void disable12V()
{
  digitalWrite(PIN_12V_EN, LOW);
}

// Prepare system for sleep
void prepareForSleep()
{
#if !INSOMNIAC
  DEBUG_PRINTLN("Info - Entering deep sleep...");
  DEBUG_PRINTLN();

  // Disable serial
  disableSerial();
#endif

  // Clear online union
  memset(&online, 0, sizeof(online));

  // Clear timer union
  memset(&timer, 0, sizeof(timer));
}

// Enter deep sleep
void goToSleep()
{
#if !INSOMNIAC
  // Enter deep sleep
  LowPower.deepSleep();
#endif

  /* Code sleeps here and awaits RTC or WDT interrupt */
}

// Wake from deep sleep
void wakeUp()
{
#if !INSOMNIAC
  // Enable serial port
  enableSerial();
#endif
}

// Non-blocking blink LED (https://forum.arduino.cc/index.php?topic=503368.0)
void blinkLed(byte ledPin, byte ledFlashes, unsigned int ledDelay)
{
  byte i = 0;
  while (i < ledFlashes * 2)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledDelay)
    {
      digitalWrite(ledPin, !digitalRead(ledPin));
      previousMillis = currentMillis;
      i++;
    }
  }
  // Ensure LED is off at end of blink cycle
  digitalWrite(ledPin, LOW);
}

// Blocking delay (in milliseconds) -- will yield and call petDog automatically
// https://arduino.stackexchange.com/questions/12587/how-can-i-handle-the-millis-rollover
void myDelay(unsigned long ms)
{
  unsigned long start, pet, now;
  start = pet = now = millis();
  while (now - start < ms)
  {
    if (now - pet >= 100) // Call petDog() every 100ms or so.
    {
      pet = now;
      if (now - start + 10 <= ms)
      {
        petDog(); // Reset watchdog timer (can take up to 10ms).
      }
    }
    else
    {
      yield(); // Allow cooperative multitasking (if used).
    }
    now = millis();
  }
}

// Force WDT to reset system (assuming WDT is configured, else it's an infinite loop)
void forceReset()
{
  DEBUG_PRINTLN("Info - Forced system reset started");
  blinkLed(PIN_LED_RED, 4, 500);
  digitalWrite(PIN_LED_RED, HIGH); // Turn on red LED
  while (true); // Wait for Watchdog Timer to reset system
}
