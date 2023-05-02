void printBmeValues() {
  //Imprime les valeurs du bme280 dans le port série.
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
