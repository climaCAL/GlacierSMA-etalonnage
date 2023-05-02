void printRtcValues(DateTime rtcTime) {
  //Récupération du temps du ds3231

  Serial.println(rtcTime.unixtime());
  Serial.print(rtcTime.year());
  Serial.print('/');
  Serial.print(rtcTime.month());
  Serial.print('/');
  Serial.print(rtcTime.day());
  Serial.print(' ');
  Serial.print(rtcTime.hour());
  Serial.print(':');
  Serial.print(rtcTime.minute());
  Serial.print(':');
  Serial.print(rtcTime.second());
  Serial.println();
}
