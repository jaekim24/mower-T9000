#include <Wire.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 12); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

#define SLAVE_ADDR 9
byte x ; 


//this is a slave
//motor 2 is faster than motor 1; motor2 +6 motor1 +0 == same speed 


void setup(){
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
  SWSerial.begin(9600);

}

void receiveEvent(){
  while(0<Wire.available()){
     x = Wire.read(); 
    Serial.println(x); 
  }
}


//if too much to the left then left motor speeds up
//if too much to the right then right motor speeds up 
void loop(){
  if (x > 5 && x <100 ){
    ST.motor(1,30);
    Serial.println("---->");
  }
  else if (x <250 && x >200){
    ST.motor(2,30); 
    Serial.println("---->");
  }
  else{
    ST.motor(1,10);
    ST.motor(2,10);
  }
  
}
