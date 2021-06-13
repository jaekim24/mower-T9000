
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 12); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

#define SLAVE_ADDR 9
byte x ; 
int motor1_speed = 46;
int motor2_speed = 40; 
boolean flag = true; 

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
    //Serial.println(x);  for debugging
  }
}

 
void loop(){
  driver();
}

void driver(){
  ST.motor(1,motor1_speed);
  ST.motor(2,motor2_speed);
  while(millis()<= 10000){
      go_straight();
      //Serial.println("repeating");  for debugging
  }
  stop();
}



//if too much to the left then left motor speeds up
//if too much to the right then right motor speeds up
void go_straight(){
  if (x > 3 && x <127 ){
    ST.motor(1,motor1_speed + 45);
    //ST.motor(1,0);  idea one: one motor stops other goes reverse
    //ST.motor(2,-30);
    Serial.println("---->");
  }
  else if (x <252 && x >128){
    ST.motor(2,motor2_speed + 39); 
    //ST.motor(2,0);  idea one: one motor stops other goes reverse
    //ST.motor(1,-30);
   Serial.println("<----");
  }
  else{
    ST.motor(1,motor1_speed);
    ST.motor(2,motor2_speed);
  }
}


void stop(){
  //Serial.println("stopping");   for debugging
  
  while(flag == true){
  ST.motor(1,0);
  ST.motor(2,0);
}
}
