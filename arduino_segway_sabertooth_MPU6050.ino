// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ------------ program to calculate the angle of inclination from the gyro and acceleration data using a Kalman filter ----------------------------------
// -------------------------- and PID control for serial control of the two motors using Sabertooth motor drivers -----------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------


#include  "Wire.h" 
#include  "I2Cdev.h"   | I2Cdev and MPU6050 must be installed as libraries
#include  "MPU6050.h"   = class default I2C address is 0x68 = AD0 low
#include  <math.h> 
#include  <SoftwareSerial.h> 
#include  <SabertoothSimplified.h> 

SoftwareSerial SWSerial(NOT_A_PIN, 11);  / RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);  Use SWSerial as the serial port.

MPU6050 accelgyro;

int16_t ax, ay, az;  / Acceleration values in x,y and z direction of the MPU6050 sensor
int16_t gx, gy, gz;  / Angular velocity values in x,y and z direction of the MPU6050 sensor

#define Pin_Lenkung_rechts 12  / Pin connector for the steering command right
#define Pin_Lenkung_links 13  / Pin connector for the steering command Left

#define Pin_Schalter_Box 3  / Pin connector of the box switch to choose between motor synchronization and I-control

#define Pin_PID_Regelung_P A1  / Pin connector for the potentiometer to change the P-content
#define Pin_PID_Regelung_I A2  / Pin connector for the potentiometer to change the I-proportion
#define Pin_PID_Regelung_D A3  / Pin connector for the potentiometer to change the D-content


int LoopTime_Soll = 9;  / desired loop duration in ms to get to the 100 Hz
int LoopTime_Angepasst = LoopTime_Soll;  / last loop time with forced break
int LoopTime_Bisher = LoopTime_Soll;  / last loop time without forced break
unsigned long LoopTime_Start = 0;  / Start time of the loop

float angle;  / current angle of inclination
float Winkel_Soll;  / Setpoint of the angle of inclination, i.e. 0°
float Winkel_Grenze;  / maximum permitted angle of inclination above which the Segway is switched off

float ACC_angle;  / Angle from the accelerometer
float GYRO_rate;  / Angular velocity from the gyro sensor

float Kp,Ki,Kd,K;  - Constants for PID control, different part, integral part, differential part, total part
int engine;  / value obtained from the PID control for motor control
int Motor_rechts, Motor_links;  / Values for the two engines
float K_Motor_links, K_Motor_rechts;  / Correction factors for synchronous running of the two motors

int Schalter_Box;  / Variable that queries the switch position on the box

int Lenkung_Eingang_rechts = 0;  / Variable for capturing a steering command to the right
int Lenkung_Eingang_links = 0;  / Variable for capturing a steering command to the left
float Lenkung_max;  / Value by which the motor control should change to the maximum in the case of a steering command
float Lenkung_rechts, Lenkung_links;  / current and gradually increased control value when steering to the right or left


// ****************************************************************************
// SETUP ******************************************
// ****************************************************************************


void setup()
   {
 Wire. begin(); 
     
    //SWSerial.begin(9600); This is the baud rate you chose with the DIP switches.
     
 Serial. begin(9600);  / baud rate for the serial monitor to check the values
    
    // initialize device
 accelgyro. initialize();
            
    calibrateSensors();  / Subroutine for one-time calibration of the sensors
   }





// ***********************************************************************************
// Calibration ***************************************************
// ***********************************************************************************


void calibrateSensors()  / one-time determination of the sensor null values (average value of 50 measurements each)
   {
    
    // =================================================
    // ======== Sensor resolution change ==========
    // =================================================
    
    // ===========================================================================
    // read raw accel/gyro measurements from device
    // Output values-Acc: Resolution 2g: 16384/g Resolution 4g: 8192/g
    // Output value gyro: Resolution 250°/s: 131/°/s Resolution 500°/s: 65.5/°/s
    // ===========================================================================
    
 accelgyro. setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
 accelgyro. setFullScaleGyroRange(MPU6050_GYRO_FS_500);
       
     
    
    // Attempt to avoid the "howling" of the two engines at the beginning
    
    ST.motor(1, 0);
    ST.motor(2, 0);
    
   
        
 Winkel_Soll = 0.0;  / Setpoint for the angle of inclination
 Winkel_Grenze = 30.0;  / maximum allowed angle of inclination
    
    
    // *******************************************************
    // K - Values for the PID control *************
    // *******************************************************
    
    
 Kp = analogRead(Pin_PID_Regelung_P) * 25.0 / 1023.0;  / Difference share fixed with potentiometer
 Ki = 0.1;  / Integral part fixed with potentiometer (switch but possibly to motor correction, therefore initially fixed with 0.1)
 Kd = analogRead(Pin_PID_Regelung_D) * 100.0 / 1023.0;  / Differential fraction fixed with potentiometer
 K = 1.0;  / Total share


    // **************************************************
    // K - Values for the engines *************
    // **************************************************


    pinMode(Pin_Schalter_Box, INPUT);  / Pin for the selection between I-control and motor synchronization
    
 K_Motor_rechts = 1.0;  / Correction factor for the right engine
 K_Motor_links = 0.8;  / Correction factor for the left engine
      
    
    // **********************************************
    // Values for steering *************
    // **********************************************
    
    
 Lenkung_max = 25.0;  / Value by which the motor control should maximally change in a steering command
 Lenkung_rechts = 0.0;  / current additional value during the steering process to the right
 Lenkung_links = 0.0;  / current additional value during the steering process to the left
 
    pinMode(Pin_Lenkung_rechts,INPUT);  / Pin for steering to the right is declared as input
    pinMode(Pin_Lenkung_links, INPUT);  / Pin for steering to the left is declared as input
      
   }






// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************
// MAIN LOOP ***************************************************************
// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************

void loop()
   {

   // *******************************************************
   // Sensor query **********************
   // *******************************************************
   
    // ===========================================================================
    // read raw accel/gyro measurements from device
    // Output values-Acc: Resolution 2g: 16384/g Resolution 4g: 8192/g
    // Output value gyro: Resolution 250°/s: 131/°/s Resolution 500°/s: 65.5/°/s
    // ===========================================================================
    
 accelgyro. getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
 ACC_angle = atan(ay * 1.0 / az * 1.0) * 180.0 / 3.141592654;  // Resolution 2g: 16384/g
   
    //ACC_angle = atan((ay/16384.0) / (az/16384.0)) * 180.0 / 3.141592654; Resolution 2g: 16384/g
    
 GYRO_rate = gx/65.5;  // Resolution 500°/s: 65.5/°/s

     
     
   // *******************************************************
   // K - Values for the PID control *************
   // *******************************************************
    
    
 Kp = analogRead(Pin_PID_Regelung_P) * 25.0 / 1023.0;  / difference share fixed with potentiometer; Maximum = 25
 Kd = analogRead(Pin_PID_Regelung_D) * 100.0 / 1023.0;  / Differential fraction fixed with potentiometer; Maximum = 100

 Schalter_Box = digitalRead(Pin_Schalter_Box);  / Query the pin for the switch state on the box
    
    if (Schalter_Box == HIGH)  = Activated by means of a switch on the Box I control
       {
 Ki = analogRead(Pin_PID_Regelung_I) * 2.0 / 1023.0;  / Integral part fixed with potentiometer; Maximum = 2
       }
    else  / By means of switch on the box motor control activated
        {
 K_Motor_rechts = analogRead(Pin_PID_Regelung_I)* 2.0 / 1023.0;  / Correction factor for synchronous running of the two motors; Maximum = 2
        }

      
     
     
     // ********************************************************************************
     // Kalman filter, PWM calculation and engine values *****************
     // ********************************************************************************
     
     
 angle = kalmanCalculate(ACC_angle, GYRO_rate, LoopTime_Angepasst);  / angle calculated with Kalman filter

     
     if (angle > Winkel_Grenze || Angle < (Winkel_Grenze * (-1)))
        {
         // ===============================================
         // Demolition due to the too large angle of inclination!
         // ===============================================
         
         ST.motor(1, 0);
         ST.motor(2, 0);
        }
     else
        {
         // =========================
         // Angle of inclination fine
         // =========================
      
 
 motor = pid(angle, Winkel_Soll, GYRO_rate);  / Calculation of the PWM value for controlling the motors
     
 Motor_rechts = K_Motor_rechts * engine;  / Calculation of the K-factor synchronized engine speed for the right engine

 Motor_links = K_Motor_links * engine;  / Calculation of the K-factor synchronized engine speed for the left engine
     
         
          
         // **************************************************************************************
         // Query whether the steering has been actuated and change of the engine control *****
         // **************************************************************************************
     
     
 Lenkung_Eingang_rechts = digitalRead(Pin_Lenkung_rechts);  / Query the pin for steering to the right

         if (Lenkung_Eingang_rechts == HIGH)
            {     
              // ******************************************
              // Steering to the right was pressed ***
              // ******************************************
          
              if (Motor_rechts >= 0)  / segway moves straight forward or stands. Which engine is queried does not matter.
                 {
 Motor_rechts = Motor_rechts - (int)Lenkung_rechts;  / Maybe also try by multiplying a factor (e.B. * (1 - 0.1)
 Motor_links = Motor_links + (int)Lenkung_rechts;  / Maybe also try by multiplying a factor (e.B. * (1 + 0.1))
                 }
              else  / segway is driving backwards
                 {
 Motor_rechts = Motor_rechts + (int)Lenkung_rechts;  / Maybe also try by multiplying a factor (e.B. * (1 + 0.1)
 Motor_links = Motor_links - (int)Lenkung_rechts;  / Maybe also try by multiplying a factor (e.B. * (1 - 0.1)
                 }
                 
 Lenkung_rechts = Lenkung_rechts + 0.05;  / Better only increase by z.B. 0.1 per query, so that steering is not too abrupt!
             
             if (Lenkung_rechts > Lenkung_max) Lenkung_rechts = Lenkung_max;  - Maximum steering value must not be exceeded!
             
             //Lenkung_rechts = constrain(Lenkung_rechts, 0, Lenkung_max); right steering value brought into the interval [0.Lenkung_max]
            } 
         else
            {
             Lenkung_rechts = 0.0;
            }
    
    
 Lenkung_Eingang_links = digitalRead(Pin_Lenkung_links);  / Query the pin for steering to the left

         if (Lenkung_Eingang_links == HIGH)
            {     
              // *****************************************
              // Steering to the left was pressed ***
              // *****************************************
          
              if (Motor_links >= 0)  / segway moves straight forward or stands. Which engine is queried does not matter.
                 {
 Motor_rechts = Motor_rechts + (int)Lenkung_links;  / Maybe also try by multiplying a factor (e.B. * (1 + 0.1)
 Motor_links = Motor_links - (int)Lenkung_links;  / Maybe also try by multiplying a factor (e.B. * (1 - 0.1)
                 }
              else  / segway is driving backwards
                 {
 Motor_rechts = Motor_rechts - (int)Lenkung_links;  / Maybe also try by multiplying a factor (e.B. * (1 - 0.1)
 Motor_links = Motor_links + (int)Lenkung_links;  / Maybe also try by multiplying a factor (e.B. * (1 + 0.1)
                 }
                 
 Lenkung_links = Lenkung_links + 0.05;  / Better only increase by z.B. 0.1 per query, so that steering is not too abrupt!
             
             if (Lenkung_links > Lenkung_max) Lenkung_links = Lenkung_max;  - Maximum steering value must not be exceeded!
             
             //Lenkung_links = constrain(Lenkung_links, 0, Lenkung_max); left steering value brought into the interval [0.Lenkung_max]
            } 
         else
            {
             Lenkung_links = 0.0;
            }
       
        
        
     
         // *******************************************************************************************
         // Control of the engines *********************************
         // *******************************************************************************************
        
         
 Motor_rechts = constrain(Motor_rechts, -127, 127);  / right motor value brought into the interval [-127,127]
 Motor_links = constrain(Motor_links, -127, 127);  / left engine value brought into the interval [-127,127]
         
                      
     /*
 Use of a root function instead of the linear control function to improve the response at low engine values
         // ======================================================================================================================================
         
 if (Motor_rechts >= 0) = right motor turns forward
            { 
 Motor_rechts = sqrt(127 * Motor_rechts); to improve the response at low engine values
              
             ST.motor(2, Motor_rechts);      
            }
 else / right motor turns backwards
            {
 Motor_rechts = -sqrt(127 * -Motor_rechts); to improve the response at low engine values
             
             ST.motor(2, Motor_rechts);               
            }
 if (Motor_links >= 0) = left motor turns forward
            {
 Motor_links = sqrt(127 * Motor_links); to improve the response at low engine values 
             
             ST.motor(1, Motor_links);               
            }
 else / left motor turns backwards
            {
 Motor_links = -sqrt(127 * -Motor_links); to improve the response at low engine values 
             
             ST.motor(1, Motor_links);  
            }
         */
         
         ST.motor(1, Motor_links);
         ST.motor(2, Motor_rechts);
         
        } 


   // ************************************************************************ 
   // Output of measured values ***************************
   // ************************************************************************

    Value output();


   // ******************************************************************
   // Keyboard query ******************************
   // ******************************************************************

   // keystroke();



   // **********************************************************************
   // loop timing control ******************************
   // **********************************************************************

 LoopTime_Bisher = millis() - LoopTime_Start;  / Time since the last loop
     
     if(LoopTime_Bisher < LoopTime_Soll)
        {
         delay(LoopTime_Soll - LoopTime_Bisher);  / Delay to get the same loop time
        }
     
 LoopTime_Angepasst = millis() - LoopTime_Start;  / updated duration of the last loop, should be equal to LoopTime_Soll = z.B. 10 msek!
 LoopTime_Start = millis();  / new starting side of the loop
   
 }


// ********************************************************************************************
// Value output to the serial interface ******************************
// ********************************************************************************************

void value output()
   {
    /*
 Serial.print(angle);
    Serial.print("     ");
 Serial.println(engine);
    Serial.print("     ");
    */
    
 Serial. print("a_y = " );
 Serial. print(ay/16384.0);
 Serial. print(" a_z = " );
 Serial. print(az/16384.0);
 Serial. print(" ACC_angle = " );
 Serial. print(ACC_angle,0);
 Serial. print(" GYRO_rate = " );
 Serial. print(GYRO_rate,0);
 Serial. print(" Angle: " );
 Serial. println(angle,0);
    
    /*
 Serial.print(" engine: ");
 Serial.print(engine);
    Serial.print("    Motor_rechts: ");
    Serial.print(Motor_rechts);
    Serial.print("    Motor_links: ");
    Serial.println(Motor_links);
    */
    
   }


// ******************************************************************************************************
// PID control ************************************
// ******************************************************************************************************

float error;
float last_error = 0;
float pTerm;
float iTerm;
float dTerm;
float integrated_error = 0;
int GUARD_GAIN = 40;  / maximally integrated angle error

   int pid(float Winkel_aktuell, float Winkel_Vorgabe, float angular velocity)
      {
       error = Winkel_Vorgabe - Winkel_aktuell;
       
 pTerm = Kp * error;  / Difference share
       
       
       integrated_error = integrated_error + error;
   
 iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);  / Integral part
  
       
 dTerm = Kd * Angular velocity / 100.0;  / Differential fraction; :100 to get to usable values!
       
       /*
       Serial.print("    K_p: ");
       Serial.print(pTerm);
       Serial.print("    K_d: ");
       Serial.println(dTerm);
       */
  
       last_error = error;
  
       // Serial.println(K*(pTerm + iTerm + dTerm));
       
  
       return constrain(K*(pTerm + iTerm + dTerm), -127, 127);  / Output of the engine value for the two engines within the limits [-127,127]
      } 

 
 

// ******************************************************************************************************
// Kalman filter module ************************************************
// ******************************************************************************************************


    float Q_angle = 0.001;  / E(alpha2) = 0.001
    float Q_gyro = 0.003;  / E(bias2) = 0.003
    float R_angle = 0.001;  / Sz = 0.03 !!! the larger the number, the more insensitive the angle reacts to changes !!!
    float x_angle = 0;
    float x_bias = 0;
    float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
    float dt, y, S;
    float K_0, K_1;

  float kalmanCalculate(float newAngle, float newRate, int looptime)
     {
 dt = float(looptime)/1000;  / dt in seconds
      x_angle = x_angle + dt * (newRate - x_bias);
      P_00 = P_00 - dt * (P_10 + P_01) + Q_angle * dt;
      P_01 = P_01 - dt * P_11;
      P_10 = P_10 - dt * P_11;
      P_11 = P_11 + Q_gyro * dt;

      y = newAngle - x_angle;
      S = P_00 + R_angle;
      K_0 = P_00 / S;
      K_1 = P_10 / S;

      x_angle +=  K_0 * y;
      x_bias  +=  K_1 * y;
      P_00 -= K_0 * P_00;
      P_01 -= K_0 * P_01;
      P_10 -= K_1 * P_00;
      P_11 -= K_1 * P_01;

      return x_angle;
     }



// ********************************************************************
// Keyboard query to change pui parameters *********
// ********************************************************************

int keystroke()
   {
    if(! Serial. available()) return 0;
   
    char param = Serial. read();  / get parameter byte
  
    if(! Serial. available()) return 0;
  
    char cmd = Serial. read();  / get command byte
  
 Serial. flush();
  
    switch (param)
       {
        case  'p':
           if(cmd=='+')    Kp++;
           if(cmd=='-')    Kp--;
           break;
        case  'i':
           if(cmd=='+')    Ki += 0.1;
           if(cmd=='-')    Ki -= 0.1;
           break;
        case  'd':
           if(cmd=='+')    Kd++;
           if(cmd=='-')    Kd--;
           break;
       case  'k':
           if(cmd=='+')    K += 0.2;
           if(cmd=='-')    K -= 0.2;
           break;
       case  'l':
           if(cmd=='+')    K_Motor_links += 0.1;
           if(cmd=='-')    K_Motor_links -= 0.1;
           break;
       case  'r':
           if(cmd=='+')    K_Motor_rechts += 0.1;
           if(cmd=='-')    K_Motor_rechts -= 0.1;
           break;
     
       default:
 Serial. print("?"); Serial. print(param);
 Serial. print(" ?"); Serial. println(cmd);
      }
  
 Serial. println();
 Serial. print("K:"); Serial. print(K);
 Serial. print(" Kp:"); Serial. print(Kp);
 Serial. print(" Ki:"); Serial. print(Ki);
 Serial. print(" Kd:"); Serial. print(Kd);
 Serial. print(" K_Motor_links:"); Serial. print(K_Motor_links);
 Serial. print(" K_Motor_rechts:"); Serial. println(K_Motor_rechts);
   } 
