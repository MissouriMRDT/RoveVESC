/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

float desiredRPM = 5000; /** The current in amps */

void setup() {
  Serial.begin(9600);


  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(19200);
  
  while (!Serial1);

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
}

void loop() {
  
  /** Call the function setCurrent() to set the motor current */
  UART.setRPM(desiredRPM);  
}