void logData(String fn, String data2Log) {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(fn, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(data2Log);
    dataFile.close();
    digitalWrite(EXT_REDLED, LOW);
    digitalWrite(EXT_GREENLED, HIGH);
    // print to the serial port too:
    Serial.println(data2Log);
  }
  // if the file isn't open, pop up an error:
  else {
    digitalWrite(EXT_REDLED, HIGH);
    digitalWrite(EXT_GREENLED, LOW);
    Serial.println("error opening datalog.txt");
  }
}
