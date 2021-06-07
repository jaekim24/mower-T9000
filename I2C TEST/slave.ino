#include <Wire.h>
#include <MPU6050_light.h>

void setup(){
    Wire.begin(8); // join i2c bus with address #8
    Serial.begin(9600); // starts the serial for output
}

void loop(){
    mpu.update();
    char z = Wire.read();
    Serial.print(z);

}
