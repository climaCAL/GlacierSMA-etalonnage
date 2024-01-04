// Configure microSD
void configureSd()
{
#if LOGGING
  // Check if microSD is force disabled
  if (disabled.microSd) {
    DEBUG_PRINTLN("Info - microSD disabled.");
    return;
  }

  // Check if microSD has already been initialized
  if (online.microSd) {
    DEBUG_PRINTLN("Info - microSD already initialized.");
    return;
  }

  // Initialize microSD
  if (!sd.begin(PIN_MICROSD_CS, SD_SCK_MHZ(12))) {
    // Try again if it failed
    DEBUG_PRINT("Warning - microSD failed to initialize (error code ");
    DEBUG_PRINT_HEX(sd.sdErrorCode());
    DEBUG_PRINTLN("); Reattempting...");

    // Delay between initialization attempts
    myDelay(2000);

    if (!sd.begin(PIN_MICROSD_CS, SD_SCK_MHZ(4))) {
      online.microSd = false;
      DEBUG_PRINTLN("Warning - microSD failed to initialize again.");
      return;
    }
  }

  online.microSd = true; // Set online flag
  DEBUG_PRINTLN("Info - microSD initialized.");
#endif
}

// Create timestamped log file
void createLogFile()
{
#if LOGGING
  // Get timestamp log file name
  //TODO If the shortest log duration is "daily", we don't really need the time in the file name;
  //     It does mean we (almost) always create a new file, but do we even want that behavior?
  sprintf(logFileName, "AWS_%s_20%02d-%02d-%02d_%02d-%02d-%02d.csv",
          CRYOLOGGER_ID, rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  // Check if log file is open
  if (logFile.isOpen())
    logFile.close();

  // Create a new log file and open for writing
  // O_CREAT  - Create the file if it does not exist
  // O_APPEND - Seek to the end of the file prior to each write
  // O_WRITE  - Open the file for writing
  if (!logFile.open(logFileName, O_CREAT | O_APPEND | O_WRITE))
  {
    DEBUG_PRINT(F("Warning - Failed to create log file ")); DEBUG_PRINTLN(logFileName);
    return;
  }
  else
  {
    DEBUG_PRINT(F("Info - Created log file: ")); DEBUG_PRINTLN(logFileName);
  }

  if (!logFile.isOpen()) //FIXME Didn't we just check this above when opening?
  {
    DEBUG_PRINT(F("Warning - Unable to open log file ")); DEBUG_PRINTLN(logFileName);
    return;
  }

  // Update file create timestamp
  updateFileCreate(&logFile);

  // Write header to file
  //FIXME Maybe we should skip this if the file already exists?
  logFile.println(F("sample,datetime,voltage,temperature_int,humidity_int,pressure_ext,temperature_ext,"
                    "humidity_ext,pitch,roll,wind_speed,wind_direction,solar,latitude,longitude,satellites,hdop,"
                    "online_bme280_ext,online_bme280_int,online_lsm303,online_veml7700,online_dfrws,online_gnss,"
                    "online_microSd,online_iridium,timer_readRtc,timer_readBattery,timer_bme280_ext,timer_bme280_int,"
                    "timer_lsm303,timer_veml7700,timer_dfrws,timer_readGnss,timer_writeMicroSd,timer_iridium,"
                    "transmit_status,rtc_drift,free_ram,"
                    "sampleInterval,averageInterval,transmitInterval,retransmitLimit,gnssTimeout,iridiumTimeout"));

  // Close log file
  logFile.close();

  // Reset related global variables
  currentLogFile = newLogFile();
  samplesSaved = 0;
#endif
}

byte newLogFile() {
  switch (loggingMode) {
  case 1: // Daily
    return rtc.getDay();
  case 3: // Yearly
    return rtc.getYear();
  case 2: // Monthly
  default:
    return rtc.getMonth();
  }
}

//TODO This really shouldn't be a macro but instead a template function, but it'll do for now...
#define LOG_PRINT(data) logFile.print(data); logFile.print(','); DEBUG_PRINT(data); DEBUG_PRINT(',');

// Write data to log file
void logData()
{
#if LOGGING
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Configure microSD
  configureSd();

  // Check if microSD is online
  if (!online.microSd) {
    DEBUG_PRINTLN(F("Warning - Logging failed since microSD is offline!"));
  }
  else {
    // Check if a new log file should be created
    if (newLogFile() != currentLogFile) {
      createLogFile();
    }

    // Check if the log file is accessible
    if (!logFile.open(logFileName, O_APPEND | O_WRITE)) {
      DEBUG_PRINT(F("Warning - Unable to open log file ")); DEBUG_PRINTLN(logFileName);
    }
    else {
      // Print data to log file and serial monitor at the same time
      DEBUG_PRINT("Info - Logging data to: "); DEBUG_PRINTLN(logFileName);

      // Sensor information
      LOG_PRINT(samplesSaved);
      LOG_PRINT(dateTime);
      LOG_PRINT(voltage);
      LOG_PRINT(temperatureInt);
      LOG_PRINT(humidityInt);
      LOG_PRINT(pressureExt);
      LOG_PRINT(temperatureExt);
      LOG_PRINT(humidityExt);
      LOG_PRINT(pitch);
      LOG_PRINT(roll);
      LOG_PRINT(windSpeed);
      LOG_PRINT(windDirection);
      LOG_PRINT(solar);
      logFile.print(latitude, 6);       logFile.print(",");
      DEBUG_PRINT_DEC(latitude, 6);     DEBUG_PRINT(",");
      logFile.print(longitude, 6);      logFile.print(",");
      DEBUG_PRINT_DEC(longitude, 6);    DEBUG_PRINT(",");
      LOG_PRINT(satellites);
      LOG_PRINT(hdop);

      // Online information
      LOG_PRINT(online.bme280Ext);
      LOG_PRINT(online.bme280Int);
      LOG_PRINT(online.lsm303);
      LOG_PRINT(online.veml7700);
      LOG_PRINT(online.dfrws);
      LOG_PRINT(online.gnss);
      LOG_PRINT(online.microSd);
      LOG_PRINT(online.iridium);

      // Timer information
      LOG_PRINT(timer.readRtc);
      LOG_PRINT(timer.readBattery);
      LOG_PRINT(timer.readBme280Ext);
      LOG_PRINT(timer.readBme280Int);
      LOG_PRINT(timer.readLsm303);
      LOG_PRINT(timer.readVeml7700);
      //LOG_PRINT(timer.readHmp60);
      //LOG_PRINT(timer.read5103L);
      //LOG_PRINT(timer.readSp212);
      LOG_PRINT(timer.readDFRWS);
      LOG_PRINT(timer.readGnss);
      LOG_PRINT(timer.writeMicroSd);
      LOG_PRINT(timer.iridium);

      // Debugging information
      LOG_PRINT(transmitStatus);
      LOG_PRINT(rtcDrift);
      LOG_PRINT(freeRam());

      // Sampling information
      LOG_PRINT(sampleInterval);
      LOG_PRINT(averageInterval);
      LOG_PRINT(transmitInterval);
      LOG_PRINT(retransmitLimit);
      LOG_PRINT(gnssTimeout);
      logFile.println(iridiumTimeout); DEBUG_PRINTLN(iridiumTimeout);

      // Update file access timestamps
      updateFileAccess(&logFile);

      // Force data to SD and update the directory entry to avoid data loss
      if (!logFile.sync()) {
        DEBUG_PRINTLN(F("Warning - microSD sync error!"));
      }

      // Close the log file
      if (!logFile.close()) {
        DEBUG_PRINTLN("Warning - Failed to close log file!");
        //closeFailCounter++; // Count number of failed file closes
      }

      blinkLed(PIN_LED_GREEN, 2, 100);
    }
  }

  //NEW We increment the samplesSaved counter whether the save actually worked or not, so that we can identify missed saves if they occur.
  samplesSaved++;

  // Stop the loop timer
  timer.writeMicroSd += millis() - loopStartTime;
#endif
}

// Update the file created timestamp
void updateFileCreate(FsFile *dataFile)
{
  if (!dataFile->timestamp(T_CREATE, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()))
  {
    DEBUG_PRINT(F("Warning - Could not update file create timestamp."));
  }
}

// Update the file access and write timestamps
void updateFileAccess(FsFile *dataFile)
{
  if (!dataFile->timestamp(T_ACCESS, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()))
  {
    DEBUG_PRINTLN(F("Warning - Could not update file access timestamp."));
  }
  if (!dataFile->timestamp(T_WRITE, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()))
  {
    DEBUG_PRINTLN(F("Warning - Could not update file write timestamp."));
  }
}
