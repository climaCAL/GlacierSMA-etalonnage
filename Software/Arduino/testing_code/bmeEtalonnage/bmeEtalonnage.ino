/*-----------------------------------------------------------
  Nom du programme : bmeEtalonnage.ino
  Réalisé par Vincent Lavallée
  Date : 2023-04-10 
  Description : Programme utilisé pour l'étalonnage de
                la température et l'humidité du BME280.
  @ApolloVL
-----------------------------------------------------------*/

//Librarie générale pour les modules i2c
#include <Wire.h>
// librairy par dualB // Claude Bouchard
#include "Ecran.h"
//Libraries pour le BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//Librairie pour le ds3231
#include "RTClib.h"
//Librairies utilisées pour l'enregistrement dans la carte uSD
#include "SD.h"
#include "SPI.h"

//Définitions
//Sortie digitale des dels de la plaquette protoTphys2V0
#define EXT_REDLED 12
#define EXT_GREENLED 13
//Niveau de la mer (bme280)
#define SEALEVELPRESSURE_HPA (1013.25)
//Chip select (cs) de al carte uSD
#define CS_SD 4

//Objets
Ecran monEcran;
Adafruit_BME280 bme; // I2C
RTC_DS3231 rtc;

//Variables
//Pour carte uSD
String fn = "bmeData.txt";
String dataTemplate = "No,Timestamp,Year,Month,Day,Hour,Minutes,Seconds,BME Temp(C),BME Hum(%)";
String dataString = "";

//Pour le "no-stoping delay"
unsigned long currentTime = millis();
uint16_t acquisitionDelay = 6000; // 6 Secondes

uint16_t acquisitionNumber = 0;

void setup() {
  Serial.begin(115200);

  //Attend une réponse du moniteur série.
  //while (!Serial);

  //Initialisation de l'objet écran.
  monEcran.begin();

  //Initialisation des dels de la plaquette proto.
  pinMode(EXT_REDLED, OUTPUT);
  pinMode(EXT_GREENLED, OUTPUT);

  //Allume les dels de la plaquette.
  ledConfirmation();

  //initialisation du bme280
  unsigned status;
  status = bme.begin();
  if (!status) {
    bmeInitError();
  } else {
    Serial.println("BME280 DETECTED");
  }

  //Initialisation du ds3231;
  if (! rtc.begin()) {
    rtcInitError();
  } else {
    Serial.println("DS3231 DETECTED");
  }
    if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CS_SD)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  //Entête du fichier qui décrit les données enregistrés.
  logData(fn, dataTemplate);
}


//"No,Timestamp,Year,Month,Day,Hour,Minutes,Seconds,BME Temp(C),BME Hum(%)"
void loop() {


  if (millis() - currentTime > acquisitionDelay) {

    monEcran.refresh();
    acquisitionNumber += 1;

    //Récupération du temps enregistrer dans le RTC
    DateTime now = rtc.now();
    printRtcValues(now);

    data2Screen(now);
    printBmeValues();

    dataString = String(acquisitionNumber) + ',' +
                 now.unixtime() + ',' +
                 now.year() + ',' +
                 now.month() + ',' +
                 now.day() + ',' +
                 now.hour() + ',' +
                 now.minute() + ',' +
                 now.second() + ',' +
                 bme.readTemperature() + ',' +
                 bme.readHumidity();

    logData(fn, dataString);
    
    currentTime = millis();
  }
}
