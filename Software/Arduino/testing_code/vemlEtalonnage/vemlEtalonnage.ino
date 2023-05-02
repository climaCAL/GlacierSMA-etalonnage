/*-----------------------------------------------------------
  Nom du programme : vemlEtalonnage.ino
  Réalisé par Vincent Lavallée
  Date : 2023-04-30 
  Description : Programme utilisé pour l'étalonnage
                du capteur de luminosité VEML7700.
  @ApolloVL
-----------------------------------------------------------*/

//--Libraries utilisées--
#include "Ecran.h"
#include "Adafruit_VEML7700.h"

//--Crréation des objets--
Ecran monEcran; //Création de l'objet 'ecran' de classe Ecran.
Adafruit_VEML7700 veml = Adafruit_VEML7700(); //Création de l'objet pour la classe du capteur de luminosité.

unsigned long currentTime = millis();
uint16_t acquisitionDelay = 500; // 1/2 Secondes

float lux; // variable de la luminosité en lux

void setup() {

  Serial.begin(115200);

  monEcran.begin(); // Initialiser l'écran.

  //Initialisation du VEML7700
  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");

}

void loop() {
  
  if (millis() - currentTime > acquisitionDelay) {

    vemlAcquire(); // Acquisition de la luminosité (LUX)

    print2screen(); // Imprime les données à l'écran

    currentTime = millis();
  }
}

//--Fonctions--
void print2screen(){

  monEcran.refresh();

  monEcran.ecrire("Luminosité :", 0); //Écrire un texte dans l'écran.
  monEcran.ecrire(String(lux), 2, 3);

  monEcran.display();

  monEcran.effacer(); //Tout effacer l'écran.

  delay(1000);

}

void vemlAcquire(){

  lux = veml.readLux(); // Default = VEML_LUX_NORMAL

  Serial.println("------------------------------------");
  Serial.print("Lux = "); Serial.println(lux);
  Serial.println("Settings used for reading:");
  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }
  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }

}
