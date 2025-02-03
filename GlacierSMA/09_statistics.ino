float nan2zero(float f) {
  return isnan(f) ? 0.0f : f;
}

// Calculate statistics
void calculateStats() //FIXME What an awful name for a function that DELETES all stats afterwards...
{
  // Write data to union
  //Yh 21-janv-2025: en appliquant de cette manière, se fie-t-on à une conversion implicite, par le compilateur?
  moSbdMessage.temperatureInt = nan2zero(temperatureIntStats.average()      * 100);     // Mean internal temperature (°C)
  moSbdMessage.humidityInt    = nan2zero(humidityIntStats.average()         * 100);     // Mean internal humidity (%)
  moSbdMessage.pressureExt    = nan2zero((pressureExtStats.average() - 400) * 100);     // Mean external pressure (hPa)
  moSbdMessage.temperatureExt = nan2zero(temperatureExtStats.average()      * 100);     // Mean external temperature (°C)
  moSbdMessage.humidityExt    = nan2zero(humidityExtStats.average()         * 100);     // Mean external humidity (%)
  moSbdMessage.solar          = nan2zero(solarStats.average()               * 10000);   // Mean solar irradiance (lx)
  //moSbdMessage.solar          = nan2zero(log10(solarStats.average()) * facteurMultLumino); // Mean solar irradiance (W/m^2), 3800*log(lx)
  moSbdMessage.hauteurNeige   = nan2zero(hauteurNeigeStats.average()        * 1);       // Hauteur de neige (mm)
  moSbdMessage.voltage        = nan2zero(batteryStats.average()             * 100);     // Mean battery voltage (V)

  // Calculate mean wind speed and direction vectors
  windVectors();

  // Clear all statistics objects
  //clearStats();

  // Clear wind gust speed and direction maximums
  windGustSpeed = 0;
  windGustDirection = 0;
  windDirectionSector = 0;

  // Write location data to union (will reuse previously stored data if readGNSS did not occur during this cycle)
  moSbdMessage.latitude = latitude * 1000000;
  moSbdMessage.longitude = longitude * 1000000;
  moSbdMessage.satellites = satellites;
  //moSbdMessage.hdop = hdop; // Message schema V0.2: This field is no longer included
}

// Clear statistics objects
void clearStats()
{
  batteryStats.clear();
  pressureIntStats.clear();
  temperatureIntStats.clear();
  humidityIntStats.clear();
  pressureExtStats.clear();
  temperatureExtStats.clear();
  humidityExtStats.clear();
  solarStats.clear();
  hauteurNeigeStats.clear();
  windSpeedStats.clear();
  uStats.clear();
  vStats.clear();

  sampleCounter = 0; // Reset sample counter
}

// Print statistics
void printStats()
{
  printLine();
  DEBUG_PRINTLN(F("Statistics"));
  printLine();
  DEBUG_PRINT("Datetime: ");      printTab(1);  printDateTime();

  DEBUG_PRINT(F("Voltage"));      printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(batteryStats.count());            printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(batteryStats.minimum());          printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(batteryStats.maximum());          printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(batteryStats.average());

  DEBUG_PRINT(F("Pressure Int"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(pressureIntStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(pressureIntStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(pressureIntStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(pressureIntStats.average());

  DEBUG_PRINT(F("Temperature Int"));                                            printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(temperatureIntStats.count());     printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(temperatureIntStats.minimum());   printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(temperatureIntStats.maximum());   printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(temperatureIntStats.average());

  DEBUG_PRINT(F("Humidity Int"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(humidityIntStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(humidityIntStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(humidityIntStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(humidityIntStats.average());

  DEBUG_PRINT(F("Pressure Ext"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(pressureExtStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(pressureExtStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(pressureExtStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(pressureExtStats.average());

  DEBUG_PRINT(F("Temperature Ext"));                                            printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(temperatureExtStats.count());     printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(temperatureExtStats.minimum());   printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(temperatureExtStats.maximum());   printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(temperatureExtStats.average());

  DEBUG_PRINT(F("Humidity Ext"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(humidityExtStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(humidityExtStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(humidityExtStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(humidityExtStats.average());

  DEBUG_PRINT(F("Solar"));                                                      printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(solarStats.count());              printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(solarStats.minimum());            printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(solarStats.maximum());            printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(solarStats.average());

  DEBUG_PRINT(F("Hauteur neige"));                                              printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(hauteurNeigeStats.count());       printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(hauteurNeigeStats.minimum());     printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(hauteurNeigeStats.maximum());     printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(hauteurNeigeStats.average());

  DEBUG_PRINT(F("Wind speed"));   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(windSpeedStats.count());          printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(windSpeedStats.minimum());        printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(windSpeedStats.maximum());        printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(windSpeedStats.average());
  DEBUG_PRINT(F("vn"));                                                         printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(vStats.count());                  printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(vStats.minimum());                printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(vStats.maximum());                printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(vStats.average());
  DEBUG_PRINT(F("ve"));                                                         printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(uStats.count());                  printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(uStats.minimum());                printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(uStats.maximum());                printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(uStats.average());
  DEBUG_PRINT(F("Wind gust speed: "));      printTab(1);  DEBUG_PRINTLN(windGustSpeed);
  DEBUG_PRINT(F("Wind gust direction: "));  printTab(1);  DEBUG_PRINTLN(windGustDirection);
}
