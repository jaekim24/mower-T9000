# MOWER-T9000
- A sub species of the Terminators, mower T9000, which it will mow your lawn for you 
![IMG_0063](https://user-images.githubusercontent.com/62858192/121279307-5c13f100-c8a2-11eb-8542-155bada366b2.jpg)


# CURRENTLY WORKING ON 
- how to use the xyz coords to move the mower with the sabertooth
    - how to make the MPU6050 dictate where the mower goes 
- make perfect 90 turns 

- [fixed] Getting the two nanos to talk to each other to use the z coordnates to keep the mower straight 
    - nano to sabertooth2x12 use TX -> S1, use serial port RX,TX for MPU6050


# NANO SPECS
<img width="1048" alt="Screen Shot 2021-09-22 at 9 53 30 AM" src="https://user-images.githubusercontent.com/62858192/134356908-e499c175-b857-4cd0-894d-752f731a3bea.png">

# PI ZERO W 
<img width="1048" alt="Screen Shot 2021-09-22 at 11 25 42 AM" src="https://user-images.githubusercontent.com/62858192/134373422-f079a43e-e8b1-4f33-836a-55936f085257.png">


# FUTURE MODS
- make a web server to see the mower move in real time

# THE PLAN 
- Get a remote comtorller path where you want it to go completely, collect that data, insert that data to the microcontrollers
- Pro : the data is easy to obtain
- Con : using the data to make the mower go to those points, how do you collect the data 
![IMG_0062 2](https://user-images.githubusercontent.com/62858192/120876274-d4ff0a00-c57d-11eb-8e36-67dbd5e5b1c8.jpg)

# USEFUL RESOURCES
- article about sending floats over I2C https://medium.com/@sandhan.sarma/sending-floats-over-i2c-between-arduinos-part-1-4e333d8ca578
- transfer float over I2C : https://www.hobbytronics.co.uk/arduino-float-vars
- issue with using nano clone with pi zero:(nano uses the /dev/ttyUSB* port, arduino uses /dev/ttyACM*) resources that helped fix it : https://marksbench.com/electronics/getting-clone-3rd-party-arduino-32u4-boards-working-on-the-raspberry-pi-linux/
- video on excuting .ino with pi in terminal https://www.youtube.com/watch?v=qAM2S27FWAI&list=LL&index=3&t=630s
- flashing a third party nano with an arduino uno https://support.arduino.cc/hc/en-us/articles/360012048100-How-to-burn-the-bootloader-in-an-Arduino-Nano-using-an-Arduino-UNO


# MATH USEFUL RESOURCES
-Finding angle between two points  https://www.cuemath.com/geometry/angle-between-vectors/
- distance formula https://courses.lumenlearning.com/waymakercollegealgebra/chapter/distance-in-the-plane/
-https://www.youtube.com/watch?v=DHPfoqiE4yQ
