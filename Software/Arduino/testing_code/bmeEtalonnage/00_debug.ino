void bmeInitError() {
  //Message d'erreur en cas ou l'initiation du bme280 à échoué.
  Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
  Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  Serial.print("        ID of 0x60 represents a BME 280.\n");
  Serial.print("        ID of 0x61 represents a BME 680.\n");
  anyError();
  while (1) delay(10);
}
void rtcInitError() {
  Serial.println("Couldn't find RTC");
  Serial.flush();
  anyError();
  while (1) delay(10);
}
void anyError() {
  //Peu importe l'erreur, témoin lumineux et écris sur l'écran.
  monEcran.effacer();
  //Programmer la del rouge -> ouverte (D4 sur plaquette protoTphys2V0)
  monEcran.ecrire("!!!!", 0, 5);
  //Allume la del rouge
  digitalWrite(EXT_REDLED, HIGH);


}
void ledConfirmation() {
  //Clignote les dels pour vérifier son fonctionnement.
  digitalWrite(EXT_REDLED, HIGH);
  digitalWrite(EXT_GREENLED, HIGH);
  delay(1000);

  digitalWrite(EXT_REDLED, LOW);
  digitalWrite(EXT_GREENLED, LOW);
}
