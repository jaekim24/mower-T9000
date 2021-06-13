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

  //dtostrf() converts float to byte enabling it transmit float over I2C
  /*format: dtostrf(desired float number, total bytes that will be printed
   *        meaning total length of the string, number of bytes after decimal, 
   *        the array to store the results)
   */ 
  dtostrf(z,7, 2, outstr);
  Wire.write(outstr);
  Serial.println(outstr); 
  Wire.endTransmission();
  //Serial.println("message sent"); 
}

