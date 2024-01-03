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

  // Set initial RTC alarm time
  rtc.setAlarmTime(0, sampleInterval, 0); // hours, minutes, seconds

  // Enable alarm for hour rollover match
  //rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.enableAlarm(rtc.MATCH_SS);  //Yh 281223: réveil dans la prochaine minute...

  // Attach alarm interrupt service routine (ISR)
  rtc.attachInterrupt(alarmIsr);

  alarmFlag = false; // Clear flag

  DEBUG_PRINT("Info - (cfgRtc) RTC initialized: "); printDateTime();
  DEBUG_PRINT("Info - (cfgRtc) Initial alarm: "); printAlarm();
  DEBUG_PRINT("Info - (cfgRtc) Alarm match "); DEBUG_PRINTLN(rtc.MATCH_SS);
}

// Read RTC
void readRtc()
{
  // Start the loop timer
  uint32_t loopStartTime = millis();

  DEBUG_PRINT("Info - (RdRtc) Current datetime: "); printDateTime();

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
  DEBUG_PRINT(F("Info - (setRtcAlrm) unixtime: ")); DEBUG_PRINTLN(unixtime);
  DEBUG_PRINT(F("Info - (setRtcAlrm) alarmTime: ")); DEBUG_PRINTLN(alarmTime);

  // Check if alarm was set in the past or too far in the future
  //Yh 281223: mis 20 afin de s'assurer d'avoir au moins 20 secondes avant le prochain cycle.
  if ((rtc.getEpoch() + 20 >= alarmTime) || ((alarmTime - unixtime) > 86400) || firstTimeFlag) {
    //Yh 281223: changé pour "Info" car n'est pas vraiment une erreur
    DEBUG_PRINTLN(F("Info - (setRtcAlrm) RTC alarm set in the past (or too tight), too far in the future, or program running for the first time."));

    // Set next alarm at a "round" minute, guaranteed to be in the future
    alarmTime = rtc.getEpoch() + min(sampleInterval * 60UL, 86400);

// Yh 281223: future work: calculer alarmTime pour ramener sur un boundary de la minute (par +/-30 secondes).

    // Reset sample counter
    sampleCounter = 0;

    // Clear all statistics objects
    clearStats(); //FIXME Do we really need to discard all collected data here?
  }
  else {
    DEBUG_PRINTLN(F("Info - (setRtcAlrm) Setting RTC alarm based on specified interval."));

//Yh 281223: reporté plus bas    rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), second(alarmTime)); // hours, minutes, seconds

  }

  // Set alarm time
  rtc.setAlarmTime(hour(alarmTime), minute(alarmTime), second(alarmTime)); // hours, minutes, seconds

  // Enable alarm for hour rollover match; Yh 281223: doit être un match MMSS afin d'éviter le cas limite qui entre en dormance pour 1 journée (cas possible avec HHMMSS)
  rtc.enableAlarm(rtc.MATCH_MMSS);


  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info - (setRtcAlrm) Current datetime: "); printDateTime();
  DEBUG_PRINT("Info - (setRtcAlrm) Next alarm: "); printAlarm();
  DEBUG_PRINT("Info - (setRtcAlrm) Alarm mode: "); DEBUG_PRINTLN(rtc.MATCH_MMSS);
}

void setCutoffAlarm() //FIXME Is this function really necessary? Why not use setRtcAlarm()?
{
  // Set next alarm at a "round" minute, guaranteed to be in the future
  alarmTime = rtc.getEpoch() + min(sampleInterval * 60UL, 86400);
  rtc.setAlarmTime(hour(alarmTime), minute(alarmTime) + 1, 0); // hours, minutes, seconds

  // Enable alarm (matching hours, minutes and seconds)
  rtc.enableAlarm(rtc.MATCH_MMSS);

  // Clear flag
  alarmFlag = false;

  DEBUG_PRINT("Info - (setCOAlrm) Current datetime: "); printDateTime();
  DEBUG_PRINT("Info - (setCOAlrm) Next alarm: "); printAlarm();
  DEBUG_PRINT("Info - (setCOAlrm) Alarm mode: "); DEBUG_PRINTLN(rtc.MATCH_MMSS);
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
  DEBUG_PRINT("Info - (chkDt) currentDate: "); DEBUG_PRINTLN(currentDate);
  DEBUG_PRINT("Info - (chkDt) newDate: "); DEBUG_PRINTLN(newDate);
}
