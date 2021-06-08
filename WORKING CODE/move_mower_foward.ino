// Software Serial Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 13); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

boolean flag = true; 



void setup()
{
  SWSerial.begin(9600);
}
void loop()
{  
  ST.motor(1,56); //+6 bc motor2 is faster than motor1
  ST.motor(2,50);  
  delay(5500);
  stop();
}

void stop(){
  Serial.println("stopping");
  
  while(flag == true){
  ST.motor(1,0);
  ST.motor(2,0);
}
}
