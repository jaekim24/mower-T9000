#include <Wire.h> 
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
byte z-coord = 0; 

void setup(){
  Serial.begin (9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");  
}

void loop(){
  Wire.beginTransmission(8); //transmits to device #8
  mpu.update();
  //Serial.print("X: ");
  //Serial.println(mpu.getAngleX());
  //Serial.print("Y: ");
  //Serial.println(mpu.getAngleY());
  Wire.write("Z: " );
  Wire.write(mpu.getAngleZ());
  Wire.endTransmission();    //stops the transmitting 
  delay(500);
}
