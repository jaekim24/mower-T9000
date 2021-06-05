
//arduino uno/nano SCL: A5 SDA: A4


#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

void setup(){
  Serial.begin(9600);
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
  mpu.update();
  //Serial.print("X: ");
  //Serial.println(mpu.getAngleX());
  //Serial.print("Y: ");
  //Serial.println(mpu.getAngleY());
  Serial.print("Z: " );
  Serial.println(mpu.getAngleZ());
  
  }
