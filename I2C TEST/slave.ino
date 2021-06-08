#include <Wire.h>
#define SLAVE_ADDR 9

//this is a slave

void setup(){
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);

}

void receiveEvent(){
  while(0<Wire.available()){
    byte x = Wire.read(); 
    Serial.println(x); 
  }
}

void loop(){
  delay(0);
  
}
