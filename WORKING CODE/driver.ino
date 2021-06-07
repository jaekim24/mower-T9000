#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2,3);

//jae 
//motor one: 64 stop
//           64 -> 127 foward
//           65 -> 96 -> 127
//           64 -> 1 reverse
//motor two: 192 stop
//           192 -> 255 foward
//           193 -> 224 -> 255
//           192 -> 128 reverse


byte m1;
int count = 0;
MPU6050 mpu(Wire);
boolean flag = true;
boolean trip = true;
boolean out = true; 
long start; 
 

void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
  mpu.calcOffsets();
  //Serial.println("Done!\n");
}

void stop(){
Serial.println("stopping");
while(flag == true){
m1 = 192;
Serial.write(m1);
m1=64;
Serial.write(m1);
}
}

void dothisfor10seconds(){
  start = millis();
  while(millis()-start<100){
    break; 
  }
}

void straight (){
  if (mpu.getAngleZ() >2){
      while(mpu.getAngleZ()>2){
        Serial.println("-------->");
        m1=100;//increases speed of left motor
        Serial.write(m1);
        mpu.update();
        }
      }
      m1= 90; // back to straight 
  
    mpu.update();
  if (mpu.getAngleZ()<-2){
      while(mpu.getAngleZ()<-2){
        Serial.println("<--------");
        m1 = 228; //increases speed of right motor
        Serial.write(m1);
        mpu.update();        
        }
      }
      m1 = 218;// back to straight 
    mpu.update();
}

void loop(){
   float z = mpu.getAngleZ();
   start = millis();
   //while( millis - start < 1000){
   mpu.update();
   m1 = 87;
   Serial.write(m1);
   m1 = 213;
   Serial.write(m1);
   delay(100);
   Serial.println(z);
   straight();
  
   }

