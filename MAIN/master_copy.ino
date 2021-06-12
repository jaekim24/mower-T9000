#include <Wire.h>
#include <MPU6050_light.h>
#define SLAVE_ADDR 9 

MPU6050 mpu(Wire);
static char outstr[15];

//this is the master

void setup(){
  Wire.begin();
  Serial.begin(115200);
  mpu.begin();
  mpu.calcOffsets();
  Serial.println("setting up MPU6050");
}

void loop(){
  delay(40);
  mpu.update();
  float z = mpu.getAngleZ() ; 
  //Serial.println("writing to slave");
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(int(z));
  //Serial.println(int(z));
  Wire.endTransmission();
  //Serial.println("message sent"); 
}
