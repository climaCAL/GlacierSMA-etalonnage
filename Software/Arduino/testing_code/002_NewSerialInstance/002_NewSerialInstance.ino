/*

  AnalogReadSerial on new UART placed on pins 1 and 0

  Reads an analog input on pin A0, prints the result to the serial monitor.

  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)

  Attach the center pin of a potentiometer to pin A0, and the outside pins to +3.3V and ground.

  Short together pin 1 and pin 0 with a wire jumper

 Created 20 Jun 2016

 by

 Arturo Guadalupi <a.guadalupi@arduino.cc>

  This example code is in the public domain.

*/

#include <Arduino.h>
#include "wiring_private.h"

#define PIN_RX A2
#define PIN_TX A1

Uart mySerial (&sercom4, PIN_RX, PIN_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0

// the setup routine runs once when you press reset:
void setup() {

  // initialize serial communication at 9600 bits per second:

  Serial.begin(115200);

  mySerial.begin(115200);

  pinPeripheral(PIN_RX, PIO_SERCOM_ALT); //Assign RX function to pin 1

  pinPeripheral(PIN_TX, PIO_SERCOM_ALT); //Assign TX function to pin 0
}



// the loop routine runs over and over again forever:
uint8_t i=0;
void loop() {
  Serial.print(i);
  mySerial.write(i++);
  if (mySerial.available()) {
    Serial.print(" -> 0x"); Serial.print(mySerial.read(), HEX);
  }
  Serial.println();
  
  delay(100);
}

// Attach the interrupt handler to the SERCOM
void SERCOM4_Handler()
{

  mySerial.IrqHandler();
}