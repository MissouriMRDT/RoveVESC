/*
  Name:    getVescValues.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description:  This example is made using a Arduino Micro (Atmega32u4) that has a HardwareSerial port (Serial1) seperated from the Serial port. 
                A Arduino Nano or Uno that only has one Serial port will not be able to display the data returned.
*/

#include <VescUart.h>
#include "RoveComm.h"


/** Initiate VescUart class */
VescUart UART;
VescUart UART3;
VescUart UART4;
VescUart UART5;
VescUart UART6;
VescUart UART7;

/** Initiate RoveComm class */
RoveCommEthernet RoveComm;

/** Initiate RoveComm packet */
rovecomm_packet packet;

float desiredRPM = 5000; /** The current in amps */

//declare the Ethernet Server in the top level sketch with the requisite port ID any time you want to use RoveComm
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT );

void setup() {

  /** Setup Serial port to display data */
  Serial.begin(9600);

  /** Setup UART port (Serial2 on Atmega32u4) */
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);
  
  while (!(Serial2 && Serial3 && Serial4 && Serial5 && Serial6 && Serial7)) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial2);
  UART3.setSerialPort(&Serial3);
  UART4.setSerialPort(&Serial4);
  UART5.setSerialPort(&Serial5);
  UART6.setSerialPort(&Serial6);
  UART7.setSerialPort(&Serial7);


  // Set up RoveComm with testing IP
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
  delay(100);

  // Debug over USB
  //UART.setDebugPort(&Serial);
}

void loop() {
  
  packet = RoveComm.read();

  /** Call the function getVescValues() to acquire data from VESC */
  if ( UART.getVescValues() ) {

    Serial.println(UART.data.rpm);
    Serial.println(UART.data.inpVoltage);
    Serial.println(UART.data.ampHours);
    Serial.println(UART.data.tachometerAbs);

  }
  else
  {
    Serial.println("Failed to get data!");
  }
  
  switch(packet.data_id)
  {
    case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATA_ID:
      //cast the packet to the correct data type
      int16_t* speeds;
      speeds = (int16_t*)packet.data;
      Serial.println(speeds[0]);
      UART.setRPM((float)speeds[0]); 
      UART3.setRPM((float)speeds[0]); 
      UART4.setRPM((float)speeds[0]); 
      UART5.setRPM((float)speeds[0]); 
      UART6.setRPM((float)speeds[0]); 
      UART7.setRPM((float)speeds[0]); 
      break;
  }
  delay(50);
}
