void data2Screen(DateTime rtcTime) {
  monEcran.effacer();
  monEcran.ecrire("Acq. No. " + String(acquisitionNumber), 0);
  monEcran.ecrire("Time " + String(rtcTime.hour()) + ':' + String(rtcTime.minute()) + ':' + String(rtcTime.second()) , 1);
  monEcran.ecrire("TEMP(°C): " + String(bme.readTemperature()), 2);
  monEcran.ecrire("HUM(%): " + String(bme.readHumidity()), 3);
}
