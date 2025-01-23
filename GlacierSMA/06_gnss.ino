// Read GNSS
void readGnss()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  if (disabled.gnss) {
    DEBUG_PRINTLN(F("Info - GNSS module disabled."));
    return;
  }

  // Assume the GNSS module is present until proven otherwise
  online.gnss = true;
  
  // Reset GNSS fix counter
  byte fixCounter = 0;

  // Enable power to GNSS
  enableGnssPower();

  // Open serial port at 9600 baud
  GNSS_PORT.begin(9600);
  DEBUG_PRINT(F("Info - Beginning to listen for GNSS traffic"));
 
  // Configure GNSS
  // Note: a delay of at least 1 s is required after powering on GNSS module
  myDelay(1000);
  GNSS_PORT.println("$PMTK220,1000*1F"); // Set NMEA update rate to 1 Hz
  GNSS_PORT.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Set NMEA sentence output frequencies to GGA and RMC
  //GNSS_PORT.println("$PGCMD,33,1*6C"); // Enable antenna updates
  //GNSS_PORT.println("$PGCMD,33,0*6D"); // Disable antenna updates

  // Look for GNSS signal for up to gnssTimeout
  while (fixCounter < 10 && millis() - loopStartTime < gnssTimeout * 1000UL) {

    // Call ISDB callback during acquisition of GNSS fix;
    // This is unrelated to GNSS, but it resets the WDT and blinks the green LED during a potentially slow operation.
    ISBDCallback();

    // Exit loop early if no GNSS data is received after a specified duration
    if ((millis() - loopStartTime) > 5000 && gnss.charsProcessed() < 10) {
      online.gnss = false;
      break;
    }

    if (!GNSS_PORT.available())
      continue;

    char c = GNSS_PORT.read();
    #if DEBUG_GNSS
      Serial.write(c); // Echo NMEA sentences to serial
    #endif
    if (!gnss.encode(c))
      continue;

    // Check if NMEA sentences have a valid fix and are not stale
    if (!((gnssFix.value() > 0 && gnssFix.age() < 1000) &&
        (String(gnssValidity.value()) == "A" && gnssValidity.age() < 1000) &&
        gnss.satellites.value() > 0))
      continue;

    fixCounter++; // Increment fix counter
    DEBUG_PRINT(' '); DEBUG_PRINT(fixCounter);
  }
  DEBUG_PRINTLN();

  if (fixCounter < 10) {
    if (online.gnss)
      DEBUG_PRINTLN(F("Warning - Insufficient GNSS fixes found before timeout!"));
    else
      DEBUG_PRINTLN(F("Warning - No GNSS data received; Please check wiring."));
    blinkLed(PIN_LED_RED, 5, 100);
  }
  else {
    // Convert GNSS date and time to epoch time
    tm.Hour = gnss.time.hour();
    tm.Minute = gnss.time.minute();
    tm.Second = gnss.time.second();
    tm.Day = gnss.date.day();
    tm.Month = gnss.date.month();
    tm.Year = gnss.date.year() - 1970; // Offset from 1970
    unsigned long gnssEpoch = makeTime(tm); // Change the tm structure into time_t (seconds since epoch)

    // Get RTC epoch time
    unsigned long rtcEpoch = rtc.getEpoch();

    // Calculate RTC drift
    long rtcDrift = rtcEpoch - gnssEpoch;

    DEBUG_PRINT(F("Info - gnssEpoch: ")); DEBUG_PRINTLN(gnssEpoch);
    DEBUG_PRINT(F("Info - rtcEpoch: ")); DEBUG_PRINTLN(rtcEpoch);

    // Sync RTC with GNSS only if gnssEpoch is greater than current unixtime
    if ((gnssEpoch > unixtime) || firstTimeFlag) {
      rtc.setEpoch(gnssEpoch);
      DEBUG_PRINT(F("Info - RTC synced to: ")); printDateTime();
    }
    else {
      DEBUG_PRINT(F("Warning - RTC sync failed. GNSS time not accurate! ")); printDateTime();
    }

    // Record position information
    latitude = gnss.location.lat();
    longitude = gnss.location.lng();
    satellites = gnss.satellites.value();
    hdop = gnss.hdop.value();

    DEBUG_PRINT(F("Info - RTC drift ")); DEBUG_PRINT(rtcDrift); DEBUG_PRINTLN(F(" seconds"));
    blinkLed(PIN_LED_GREEN, 5, 100);
  }

  // Close GNSS port
  GNSS_PORT.end();

  // Disable power to GNSS
  disableGnssPower();

  // Stop the loop timer
  timer.readGnss = millis() - loopStartTime;
}
