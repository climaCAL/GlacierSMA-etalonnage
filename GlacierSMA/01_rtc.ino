// Configure the real-time clock (RTC)
void configureRtc()
{
  // Alarm modes:
  // 0: MATCH_OFF          Never
  // 1: MATCH_SS           Every Minute
  // 2: MATCH_MMSS         Every Hour
  // 3: MATCH_HHMMSS       Every Day
  // 4: MATCH_DHHMMSS      Every Month
  // 5: MATCH_MMDDHHMMSS   Every Year
  // 6: MATCH_YYMMDDHHMMSS Once, on a specific date and a specific time

  // Initialize RTC
  rtc.begin();

  // Set time manually
  //rtc.setTime(22, 55, 0); // Must be in the form: rtc.setTime(hours, minutes, seconds);
  //rtc.setDate(27, 1, 23); // Must be in the form: rtc.setDate(day, month, year);

  // Attach alarm interrupt service routine (ISR)
  rtc.attachInterrupt(alarmIsr);

  alarmFlag = false; // Clear flag

  DEBUG_PRINT("Info - RTC initialized: "); printDateTime();
  DEBUG_PRINT("Info - Initial alarm: "); printAlarm();
}

// Read RTC
void readRtc()
{
  // Start the loop timer
  uint32_t loopStartTime = millis();

  DEBUG_PRINT("Info - Current datetime: "); printDateTime();

  // Get Unix Epoch time
  unixtime = rtc.getEpoch();

  // Write data to union
  moSbdMessage.unixtime = unixtime;

  // Stop the loop timer
  timer.readRtc = millis() - loopStartTime;
}

// Set RTC alarm
void setRtcAlarm()
{
  // Calculate next alarm
  alarmTime = unixtime + (sampleInterval * 60UL);
  DEBUG_PRINT(F("Info - unixtime: ")); DEBUG_PRINTLN(unixtime);
  DEBUG_PRINT(F("Info - alarmTime: ")); DEBUG_PRINTLN(alarmTime);

  // Check if alarm was set in the past or too far in the future
  if ((rtc.getEpoch() + 1 >= alarmTime) || ((alarmTime - unixtime) > 86400) || firstTimeFlag) {
    DEBUG_PRINTLN(F("Error - RTC alarm set in the past, too far in the future, or program running for the first time."));

    // Set next alarm at a "round" minute, guaranteed to be in the future
    alarmTime = rtc.getEpoch() + min(sampleInterval * 60UL, 86400);
    rtc.setAlarmTime(hour(alarmTime), minute(alarmTime) + 1, 0); // hours, minutes, seconds

    // Reset sample counter
    sampleCounter = 0;

    // Clear all statistics objects
    clearStats(); //FIXME Do we really need to discard all collected data here?
  }
  else {
    DEBUG_PRINTLN(F("Info - Setting RTC alarm based on specified interval."));

    // Set alarm time
    rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), second(alarmTime)); // hours, minutes, seconds
  }

  // Enable alarm (matching hours, minutes and seconds)
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info - Current datetime: "); printDateTime();
  DEBUG_PRINT("Info - Next alarm: "); printAlarm();
  DEBUG_PRINT("Info - Alarm mode: "); DEBUG_PRINTLN(rtc.MATCH_HHMMSS);
}

void setCutoffAlarm() //FIXME Is this function really necessary? Why not use setRtcAlarm()?
{
  // Set next alarm at a "round" minute, guaranteed to be in the future
  alarmTime = rtc.getEpoch() + min(sampleInterval * 60UL, 86400);
  rtc.setAlarmTime(hour(alarmTime), minute(alarmTime) + 1, 0); // hours, minutes, seconds

  // Enable alarm (matching hours, minutes and seconds)
  rtc.enableAlarm(rtc.MATCH_HHMMSS);

  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info - Current datetime: "); printDateTime();
  DEBUG_PRINT("Info - Next alarm: "); printAlarm();
  DEBUG_PRINT("Info - Alarm mode: "); DEBUG_PRINTLN(rtc.MATCH_HHMMSS);
}

// RTC alarm interrupt service routine (ISR)
void alarmIsr()
{
  alarmFlag = true;
}

// Print the RTC's current date and time
void printDateTime()
{
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  DEBUG_PRINTLN(dateTimeBuffer);
}

// Print the RTC alarm
void printAlarm()
{
  char alarmBuffer[25];
  sprintf(alarmBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getAlarmYear(), rtc.getAlarmMonth(), rtc.getAlarmDay(),
          rtc.getAlarmHours(), rtc.getAlarmMinutes(), rtc.getAlarmSeconds());
  DEBUG_PRINTLN(alarmBuffer);
}


void checkDate()
{
  // Record log file tracker the first time program runs
  if (firstTimeFlag)
  {
    currentDate = rtc.getDay();
  }
  newDate = rtc.getDay();
  Serial.print("currentDate: "); Serial.println(currentDate);
  Serial.print("newDate: "); Serial.println(newDate);
}
