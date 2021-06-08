#include <Wire.h>
#define SLAVE_ADDR 9 

void setup(){
  Wire.begin();
  Serial.begin(115200);
  Serial.println("i2c master demo");
  
}
void loop(){
  delay(40);
  Serial.println("writing to slave");
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(0);
  Wire.endTransmission();
  Serial.println("message sent"); 
}
