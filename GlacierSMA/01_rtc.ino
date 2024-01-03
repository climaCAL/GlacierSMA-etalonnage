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
  rtc.begin(); // begin(bool resetTime = true) would reset time to 0 on startup

  // Set time manually
  //rtc.setTime(22, 55, 0); // Must be in the form: rtc.setTime(hours, minutes, seconds);
  //rtc.setDate(27, 1, 23); // Must be in the form: rtc.setDate(day, month, year);

  // Read RTC initially and display time
  readRtc();

  // Set initial RTC alarm time (FIXME This isn't needed unless loop() never gets called)
  setRtcAlarm();

  // Attach alarm interrupt service routine (ISR)
  rtc.attachInterrupt(alarmIsr);
}

// Read RTC
void readRtc()
{
  // Start the loop timer
  uint32_t loopStartTime = millis();

  DEBUG_PRINT("Info - (readRtc) Current datetime: ");
  printDateTime(dateTime); // This writes the date to a persistent string (used for logging)
  DEBUG_PRINTLN(dateTime);

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
  alarmTime = unixtime + sampleInterval * 60;
  alarmTime -= second(alarmTime); // Discard the seconds

  DEBUG_PRINT(F("Info - (setRtcAlarm) unixtime: ")); DEBUG_PRINTLN(unixtime);
  DEBUG_PRINT(F("Info - (setRtcAlarm) alarmTime: ")); DEBUG_PRINTLN(alarmTime);

  // Check if alarm is set in the past (or less than 5 seconds from now) or too far in the future
  if (alarmTime <= rtc.getEpoch() + 5 || alarmTime > rtc.getEpoch() + sampleInterval * 60) {
    DEBUG_PRINTLN(F("Warning - (setRtcAlarm) RTC alarm set in the past or too far in the future."));

    // Update alarm time based on current time + sample interval so it is guaranteed to be in the future
    // This is basically akin to "skipping" a sample (in the worst case)
    alarmTime = rtc.getEpoch() + min(sampleInterval * 60, 3600); // Max 1 hour since we match MM:SS
  }

  // Set next alarm at a "whole" minute
  rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0); // hours, minutes, seconds
  
  // Enable alarm for minute-and-second match
  rtc.enableAlarm(rtc.MATCH_MMSS);

  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info - (setRtcAlarm) Current datetime: "); printDateTime();
  DEBUG_PRINT("Info - (setRtcAlarm) Next alarm: "); printAlarm();
  DEBUG_PRINT("Info - (setRtcAlarm) Alarm mode: "); DEBUG_PRINTLN("MMSS");
}

void setCutoffAlarm() //FIXME Is this function really necessary? Why not reuse setRtcAlarm() with a parameter?
{
  // Set next alarm at a "whole" minute, guaranteed to be in the future
  alarmTime = rtc.getEpoch() + min(sampleInterval * 60, 3600); // Max 1 hour since we match MM:SS
  rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), 0); // hours, minutes, seconds

  // Enable alarm (matching hours, minutes and seconds)
  rtc.enableAlarm(rtc.MATCH_MMSS);

  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info - (setCOAlrm) Current datetime: "); printDateTime();
  DEBUG_PRINT("Info - (setCOAlrm) Next alarm: "); printAlarm();
  DEBUG_PRINT("Info - (setCOAlrm) Alarm mode: "); DEBUG_PRINTLN("MMSS");
}

// Check that the next alarm is set correctly;
//TODO Unfortunately there is no method to check that the alarm is enabled in the first place...
bool checkAlarm()
{
  return alarmTime > rtc.getEpoch() && alarmTime < rtc.getEpoch() + sampleInterval * 60;
}

// RTC alarm interrupt service routine (ISR)
void alarmIsr()
{
  alarmFlag = true;
}

// Print the RTC's current date and time
void printDateTime()
{
  char dateTimeBuffer[20];
  printDateTime(dateTimeBuffer);
  DEBUG_PRINTLN(dateTimeBuffer);
}

//TODO Create a utility function that can print any datetime parameter, and rename this function "printCurrentTime"
void printDateTime(char* printBuffer)
{
  sprintf(printBuffer, "20%02d-%02d-%02d %02d:%02d:%02d",
          rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
}

// Print the RTC alarm
void printAlarm()
{
  char alarmBuffer[20];
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
  DEBUG_PRINT("Info - (chkDt) currentDate: "); DEBUG_PRINTLN(currentDate);
  DEBUG_PRINT("Info - (chkDt) newDate: "); DEBUG_PRINTLN(newDate);
}
