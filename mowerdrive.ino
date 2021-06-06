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
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done!\n");
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
  if (mySerial.read() >2){
      while(mySerial.read()>2 && out == true ){
        Serial.println("adjusting to the right\nIncreaseing speed+++.");
        m1=100;
        mpu.update();
        if (mySerial.read()<2){
          out = false; 
          break; 
        }
      }
      out = true; 
      Serial.println("going straight now\nSpeed back to normal.");
      m1= 90;
    }
    mpu.update();
    
  if (mySerial.read()<-2){
      while(mySerial.read()<-2 && out == true){
        Serial.println("adjusting to the left\n increasing speed+++");
        m1 = 228;
        mpu.update();
        if (mySerial.read()<2){
          out = false; 
          break; 
        }
      }
      out = true; 
      Serial.println("going straight now\nSpeed back to normal");
      m1 = 218;
    }
    mpu.update();

   
}

void loop(){
   start = millis();
   while( millis - start < 100){
   mpu.update();
   m1 = 87;
   Serial.write(m1);
   m1 = 213;
   Serial.write(m1);
   straight();
   }
   stop();  
}
