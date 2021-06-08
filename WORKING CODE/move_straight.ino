#include <Wire.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 12); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

#define SLAVE_ADDR 9
byte x ; 

//motor 2 is faster than motor 1; motor1 +6 motor2 +0 == same speed 

//this is a slave

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

 
void loop(){
  go_straight();
}


//if too much to the left then left motor speeds up
//if too much to the right then right motor speeds up
void go_straight(){
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


void stop(){
  Serial.println("stopping");
  
  while(flag == true){
  ST.motor(1,0);
  ST.motor(2,0);
}
}

