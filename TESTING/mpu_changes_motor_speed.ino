#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
SoftwareSerial SWSerial(NOT_A_PIN, 13); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.


MPU6050 mpu(Wire);
boolean flag = true; 
int motor1_speed = 56;
int motor2_speed = 50; 


void setup()
{
  SWSerial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // calibrates the sensors
}

void loop(){
  ST.motor(1,56); //+6 bc motor2 is faster than motor1
  ST.motor(2,50); 
  go_straight();
}

void go_straight (){

  int left_edge = 2;
  int right_edge = -2;
  
  if (mpu.getAngleZ() >left_edge){
      while(mpu.getAngleZ()>left_edge){
        Serial.println("-------->");
        ST.motor(1,64) ;//increases speed of left motor
        mpu.update();
        }
      }
  mpu.update();
  if (mpu.getAngleZ()<right_edge){
      while(mpu.getAngleZ()<right_edge){
        Serial.println("<--------");
        ST.motor(2,58); //increases speed of right motor
        mpu.update();        
        }
        
      }
  mpu.update();

  if (mpu.getAngleZ() < left_edge && mpu.getAngleZ() < right_edge){
    Serial.println("going straight now");
    ST.motor(1, 56);
    ST.motor(2,50);
  }
}
