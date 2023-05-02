//Broche de la del sur le micocontôleur Adafuit Feather M0 Adalogger
#define ONBOARD_LED 13

//Baudrate du ucontroleur 
const uint16_t baud = 115200;

//Délai d'allumage
const uint16_t blinkDelay = 500;

void setup() {
  //initialisation du port série
  Serial.begin(baud);
  
  //Initialisation de la led
  pinMode(ONBOARD_LED, OUTPUT);
}

void loop() {
  //Allume la del
  digitalWrite(ONBOARD_LED, HIGH);

  //Imprime un message de confirmation
  Serial.println("LED_ON");
  
  //Attend 500ms
  delay(blinkDelay);
  
  //Éteint la del
  digitalWrite(ONBOARD_LED, LOW);

    //Imprime un message de confirmation
  Serial.println("LED_OFF");
  
  //Attend 500ms
  delay(blinkDelay);
}
