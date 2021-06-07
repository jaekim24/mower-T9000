// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ------------  Programm zur Berechnung des Neigungswinkels aus den Gyro- und Beschleunigungsdaten unter Verwendung eines Kalman-Filters ----------------------------------
// --------------------------  und PID-Regelung zur seriellen Ansteuerung der beiden Motoren mittels Sabertooth-Motortreiber -----------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------


#include "Wire.h"
#include "I2Cdev.h"      // I2Cdev and MPU6050 must be installed as libraries
#include "MPU6050.h"     // class default I2C address is 0x68 = AD0 low
#include <math.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial(NOT_A_PIN, 11);          // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial);               // Use SWSerial as the serial port.

MPU6050 accelgyro;

int16_t ax, ay, az;   // Beschleunigungswerte in x-,y- und z-Richtung des MPU6050-Sensors
int16_t gx, gy, gz;   // Winkelgeschwindigkeitswerte in x-,y- und z-Richtung des MPU6050-Sensors

#define Pin_Lenkung_rechts      12               // Pin-Anschluss für den Lenkbefehl Rechts
#define Pin_Lenkung_links       13               // Pin-Anschluss für den Lenkbefehl Links

#define Pin_Schalter_Box        3                // Pin-Anschluss des Box-Schalters, um zwischen Motorsynchronisation und I-Regelung zu wählen

#define Pin_PID_Regelung_P      A1               // Pin-Anschluss für das Poti zur Veränderung des P-Anteils
#define Pin_PID_Regelung_I      A2               // Pin-Anschluss für das Poti zur Veränderung des I-Anteils
#define Pin_PID_Regelung_D      A3               // Pin-Anschluss für das Poti zur Veränderung des D-Anteils


int LoopTime_Soll = 9;                           // gewünschte Schleifendauer in ms um auf die 100 Hz zu gelangen      
int LoopTime_Angepasst = LoopTime_Soll;          // letzte Schleifenzeit mit erzwungener Pause
int LoopTime_Bisher = LoopTime_Soll;             // letzte Schleifenzeit ohne erzwungener Pause
unsigned long LoopTime_Start = 0;		 // Startzeit der Schleife

float Winkel;					 // aktueller Neigungswinkel
float Winkel_Soll;                               // Sollwert des Neigungswinkels, also 0°
float Winkel_Grenze;                             // maximal erlaubter Neigungswinkel, über dem das Segway abgeschaltet wird   

float ACC_angle;				 // Winkel vom Beschleunigungssensor
float GYRO_rate;		                 // Winkelgeschwindigkeit vom Gyro-Sensor

float Kp,Ki,Kd,K;                                // Konstanten zur PID-Regelung, Differentteil, Integralteil, Differentialteil, Gesamtanteil
int Motor;                                       // von der PID-Regelung erhaltener Wert zur Motoransteuerung
int Motor_rechts, Motor_links;                   // Werte für die beiden Motoren
float K_Motor_links, K_Motor_rechts;             // Korrekturfaktoren für einen synchronen Lauf der beiden Motoren

int Schalter_Box;                                // Variable, welche die Schalterstellung an der Box abfragt

int Lenkung_Eingang_rechts = 0;                  // Variable zum Erfassen eines Lenkbefehls nach Rechts
int Lenkung_Eingang_links = 0;                   // Variable zum Erfassen eines Lenkbefehls nach Links
float Lenkung_max;                               // Wert, um den sich die Motoransteuerung bei einem Lenkbefehl maximal ändern soll
float Lenkung_rechts, Lenkung_links;             // aktueller und schrittweise erhöhter Ansteuerungswert beim Lenken nach rechts bzw. links


// ****************************************************************************
// ****************************** SETUP ***************************************
// ****************************************************************************


void setup()
   {
    Wire.begin(); 
     
    //SWSerial.begin(9600);     // This is the baud rate you chose with the DIP switches.
     
    Serial.begin(9600);    // baud rate für den seriellen Monitor, um die Werte zu überprüfen
    
    // initialize device
    accelgyro.initialize();
            
    calibrateSensors();       // Unterprogramm zum einmaligen Kalibrieren der Sensoren
   }





// ***********************************************************************************
// ****************************** Kalibrierung ***************************************
// ***********************************************************************************


void calibrateSensors()              // einmalige Ermittlung der Sensor-Nullwerte (Mittelwert aus jeweils 50 Messungen)
   {
    
    // =================================================
    // ======== Änderung der Sensor-Auflösung ==========
    // =================================================
    
    // ===========================================================================
    // read raw accel/gyro measurements from device
    // Ausgabewerte-Acc: Auflösung 2g:  16384/g    Auflösung 4g:  8192/g
    // Ausgabewerte-Gyro: Auflösung 250°/s:  131/°/s   Auflösung 500°/s:  65.5/°/s
    // ===========================================================================
    
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
       
     
    
    // Versuch, das "Aufheulen" der beiden Motoren zu Beginn zu vermeiden
    
    ST.motor(1, 0);
    ST.motor(2, 0);
    
   
        
    Winkel_Soll = 0.0;                 // Sollwert für den Neigungswinkel
    Winkel_Grenze = 30.0;              // maximal erlaubter Neigungswinkel
    
    
    // *******************************************************
    // ********** K - Werte für die PID-Regelung *************
    // *******************************************************
    
    
    Kp = analogRead(Pin_PID_Regelung_P) * 25.0 / 1023.0;        // Differenzenanteil mit Poti festgelegt
    Ki = 0.1;                                                   // Integralanteil mit Poti festgelegt (Schalter aber womöglich auf Motorkorrektur, daher mit 0.1 zunächst festgesetzt)   
    Kd = analogRead(Pin_PID_Regelung_D) * 100.0 / 1023.0;       // Differentialanteil mit Poti festgelegt
    K = 1.0;                                                    // Gesamtanteil


    // **************************************************
    // ********** K - Werte für die Motoren *************
    // **************************************************


    pinMode(Pin_Schalter_Box, INPUT);      // Pin für die Auswahl zwischen I-Regelung und Motorsynchronisation
    
    K_Motor_rechts = 1.0;                  // Korrekturfaktor für den rechten Motor
    K_Motor_links = 0.8;                   // Korrekturfaktor für den linken Motor
      
    
    // **********************************************
    // ********** Werte für die Lenkung *************
    // **********************************************
    
    
    Lenkung_max = 25.0;                      // Wert, um den sich die Motoransteuerung bei einem Lenkbefehl MAXIMAL ändern soll
    Lenkung_rechts = 0.0;                    // aktueller Zusatzwert beim Lenkvorgang nach rechts
    Lenkung_links = 0.0;                     // aktueller Zusatzwert beim Lenkvorgang nach links
 
    pinMode(Pin_Lenkung_rechts, INPUT);      // Pin für Lenkung nach Rechts wird als Input deklariert
    pinMode(Pin_Lenkung_links, INPUT);       // Pin für Lenkung nach Links wird als Input deklariert
      
   }






// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************
// ********************************************************************** HAUPTSCHLEIFE **************************************************************
// ***************************************************************************************************************************************************
// ***************************************************************************************************************************************************

void loop()
   {

   // *******************************************************
   // ********************* Sensorabfrage *******************
   // *******************************************************
   
    // ===========================================================================
    // read raw accel/gyro measurements from device
    // Ausgabewerte-Acc: Auflösung 2g:  16384/g    Auflösung 4g:  8192/g
    // Ausgabewerte-Gyro: Auflösung 250°/s:  131/°/s   Auflösung 500°/s:  65.5/°/s
    // ===========================================================================
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   
    ACC_angle = atan(ay * 1.0 / az * 1.0) * 180.0 / 3.141592654;       // Auflösung 2g:  16384/g
   
    //ACC_angle = atan((ay/16384.0) / (az/16384.0)) * 180.0 / 3.141592654;       // Auflösung 2g:  16384/g
    
    GYRO_rate = gx/65.5;                                 // Auflösung 500°/s:  65.5/°/s

     
     
   // *******************************************************
   // ********** K - Werte für die PID-Regelung *************
   // *******************************************************
    
    
    Kp = analogRead(Pin_PID_Regelung_P) * 25.0 / 1023.0;                     // Differenzenanteil mit Poti festgelegt; Maximum = 25
    Kd = analogRead(Pin_PID_Regelung_D) * 100.0 / 1023.0;                    // Differentialanteil mit Poti festgelegt; Maximum = 100

    Schalter_Box = digitalRead(Pin_Schalter_Box);                            // Abfrage des Pins für den Schalterzustand an der Box
    
    if (Schalter_Box == HIGH)                                                // Mittels Schalter an der Box I-Regelung aktiviert
       {
        Ki = analogRead(Pin_PID_Regelung_I) * 2.0 / 1023.0;                  // Integralanteil mit Poti festgelegt; Maximum = 2        
       }
    else                                                                     // Mittels Schalter an der Box Motor-Regelung aktiviert
        {
         K_Motor_rechts = analogRead(Pin_PID_Regelung_I) * 2.0 / 1023.0;     // Korrekturfaktor für einen synchronen Lauf der beiden Motoren; Maximum = 2
        }

      
     
     
     // ********************************************************************************
     // ****************** Kalmanfilter, PWM-Berechnung und Motorwerte *****************
     // ********************************************************************************
     
     
     Winkel = kalmanCalculate(ACC_angle, GYRO_rate, LoopTime_Angepasst);	// mit Kalmanfilter berechneter Winkel

     
     if (Winkel > Winkel_Grenze || Winkel < (Winkel_Grenze * (-1)))
        {
         // ===============================================
         // Abbruch aufgrund des zu großen Neigungswinkels!
         // ===============================================
         
         ST.motor(1, 0);
         ST.motor(2, 0);
        }
     else
        {
         // =========================
         // Neigungswinkel in Ordnung
         // =========================
      
 
         Motor = pid(Winkel, Winkel_Soll, GYRO_rate);      // Berechnung des PWM-Werts zur Ansteuerung der Motoren 
     
         Motor_rechts = K_Motor_rechts * Motor;            // Berechnung der mittels K-Faktor synchronisierten Motordrehzahl für den rechten Motor

         Motor_links = K_Motor_links * Motor;              // Berechnung der mittels K-Faktor synchronisierten Motordrehzahl für den linken Motor
     
         
          
         // **************************************************************************************
         // ***** Abfrage ob die Lenkung betätigt wurde und Veränderung der Motoransteuerung *****
         // **************************************************************************************
     
     
         Lenkung_Eingang_rechts = digitalRead(Pin_Lenkung_rechts);   // Abfrage des Pins für Lenkung nach Rechts

         if (Lenkung_Eingang_rechts == HIGH)
            {     
              // ******************************************
              // *** Lenkung nach Rechts wurde gedrückt ***
              // ******************************************
          
              if (Motor_rechts >= 0)    // segway fährt gerade vorwärts oder steht. Welcher Motor abgefragt wird, ist egal.
                 {
                  Motor_rechts = Motor_rechts - (int)Lenkung_rechts;   // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 - 0.1) versuchen
                  Motor_links = Motor_links + (int)Lenkung_rechts;     // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 + 0.1)) versuchen
                 }
              else                      // segway fährt gerade rückwärts
                 {
                  Motor_rechts = Motor_rechts + (int)Lenkung_rechts;   // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 + 0.1) versuchen
                  Motor_links = Motor_links - (int)Lenkung_rechts;     // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 - 0.1) versuchen
                 }
                 
             Lenkung_rechts = Lenkung_rechts + 0.05;                                 // Besser nur um z.B. 0.1 pro Abfrage erhöhen, damit Lenkung nicht zu abrupt erfolgt!
             
             if (Lenkung_rechts > Lenkung_max) Lenkung_rechts = Lenkung_max;        // Maximaler Lenkungswert darf nicht überschritten werden!
             
             //Lenkung_rechts = constrain(Lenkung_rechts, 0, Lenkung_max);          // rechter Lenkungswert ins Intervall [0,Lenkung_max] gebracht
            } 
         else
            {
             Lenkung_rechts = 0.0;
            }
    
    
         Lenkung_Eingang_links = digitalRead(Pin_Lenkung_links);    // Abfrage des Pins für Lenkung nach Links

         if (Lenkung_Eingang_links == HIGH)
            {     
              // *****************************************
              // *** Lenkung nach Links wurde gedrückt ***
              // *****************************************
          
              if (Motor_links >= 0)    // segway fährt gerade vorwärts oder steht. Welcher Motor abgefragt wird ist egal.
                 {
                  Motor_rechts = Motor_rechts + (int)Lenkung_links;   // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 + 0.1) versuchen
                  Motor_links = Motor_links - (int)Lenkung_links;     // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 - 0.1) versuchen
                 }
              else                      // segway fährt gerade rückwärts
                 {
                  Motor_rechts = Motor_rechts - (int)Lenkung_links;   // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 - 0.1) versuchen
                  Motor_links = Motor_links + (int)Lenkung_links;     // Vielleicht auch mit Multiplikation eines Faktors (z.B. * (1 + 0.1) versuchen
                 }
                 
             Lenkung_links = Lenkung_links + 0.05;                                 // Besser nur um z.B. 0.1 pro Abfrage erhöhen, damit Lenkung nicht zu abrupt erfolgt!
             
             if (Lenkung_links > Lenkung_max) Lenkung_links = Lenkung_max;        // Maximaler Lenkungswert darf nicht überschritten werden!
             
             //Lenkung_links = constrain(Lenkung_links, 0, Lenkung_max);          // linker Lenkungswert ins Intervall [0,Lenkung_max] gebracht
            } 
         else
            {
             Lenkung_links = 0.0;
            }
       
        
        
     
         // *******************************************************************************************
         // ******************************** Ansteuern der Motoren  ***********************************
         // *******************************************************************************************
        
         
         Motor_rechts = constrain(Motor_rechts, -127, 127);          // rechter Motorenwert ins Intervall [-127,127] gebracht
         Motor_links = constrain(Motor_links, -127, 127);            // linker Motorenwert ins Intervall [-127,127] gebracht
         
                      
     /*
         // Gebrauch einer Wurzelfunktion anstelle der linearen Ansteuerfunktion zur Verbesserung des Ansprechverhaltens bei niedrigen Motorwerten
         // ======================================================================================================================================
         
         if (Motor_rechts >= 0)     // rechter Motor dreht vorwärts
            { 
             Motor_rechts = sqrt(127 * Motor_rechts);             // zur Verbesserung des Ansprechverhaltens bei niedrigen Motorwerten
              
             ST.motor(2, Motor_rechts);      
            }
         else                       // rechter Motor dreht rückwärts
            {
             Motor_rechts = -sqrt(127 * -Motor_rechts);           // zur Verbesserung des Ansprechverhaltens bei niedrigen Motorwerten
             
             ST.motor(2, Motor_rechts);               
            }
         if (Motor_links >= 0)      // linker Motor dreht vorwärts
            {
             Motor_links = sqrt(127 * Motor_links);               // zur Verbesserung des Ansprechverhaltens bei niedrigen Motorwerten 
             
             ST.motor(1, Motor_links);               
            }
         else                       // linker Motor dreht rückwärts
            {
             Motor_links = -sqrt(127 * -Motor_links);             // zur Verbesserung des Ansprechverhaltens bei niedrigen Motorwerten 
             
             ST.motor(1, Motor_links);  
            }
         */
         
         ST.motor(1, Motor_links);
         ST.motor(2, Motor_rechts);
         
        } 


   // ************************************************************************ 
   // *********************** Ausgabe der Messwerte **************************
   // ************************************************************************

    Werteausgabe();


   // ******************************************************************
   // *********************** Tastaturabfrage **************************
   // ******************************************************************

   // Tastatureingabe();



   // **********************************************************************
   // *********************** loop timing control **************************
   // **********************************************************************

     LoopTime_Bisher = millis() - LoopTime_Start;        // Zeit seit der letzten Schleife
     
     if(LoopTime_Bisher < LoopTime_Soll)
        {
         delay(LoopTime_Soll - LoopTime_Bisher);         // Verzögerung, um die gleiche Schleifenzeit zu erhalten
        }
     
     LoopTime_Angepasst = millis() - LoopTime_Start;     // aktualisierte Dauer der letzten Schleife, sollte gleich LoopTime_Soll = z.B. 10 msek sein!
     LoopTime_Start = millis();                          // neue Starteit der Schleife
   
 }


// ********************************************************************************************
// ****************** Werteausgabe an die serielle Schnittstelle ******************************
// ********************************************************************************************

void Werteausgabe()
   {
    /*
    Serial.print(Winkel);
    Serial.print("     ");
    Serial.println(Motor);
    Serial.print("     ");
    */
    
    Serial.print("a_y = ");
    Serial.print(ay/16384.0);
    Serial.print("    a_z = ");
    Serial.print(az/16384.0);
    Serial.print("    ACC_angle = ");
    Serial.print(ACC_angle,0);
    Serial.print("    GYRO_rate = ");
    Serial.print(GYRO_rate,0);
    Serial.print("    Winkel: ");
    Serial.println(Winkel,0);
    
    /*
    Serial.print("   Motor: ");
    Serial.print(Motor);
    Serial.print("    Motor_rechts: ");
    Serial.print(Motor_rechts);
    Serial.print("    Motor_links: ");
    Serial.println(Motor_links);
    */
    
   }


// ******************************************************************************************************
// ***************************************** PID-Steuerung **********************************************
// ******************************************************************************************************

float error;
float last_error = 0;
float pTerm;
float iTerm;
float dTerm;
float integrated_error = 0;
int GUARD_GAIN = 40;           // maximal aufintegrierter Winkelfehler

   int pid(float Winkel_aktuell, float Winkel_Vorgabe, float Winkelgeschwindigkeit)
      {
       error = Winkel_Vorgabe - Winkel_aktuell;
       
       pTerm = Kp * error;                                                         // Differenzenanteil
       
       
       integrated_error = integrated_error + error;
   
       iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);          // Integralanteil
  
       
       dTerm = Kd * Winkelgeschwindigkeit / 100.0;                                 // Differentialanteil; :100 um auf brauchbare Werte zu kommen!
       
       /*
       Serial.print("    K_p: ");
       Serial.print(pTerm);
       Serial.print("    K_d: ");
       Serial.println(dTerm);
       */
  
       last_error = error;
  
       // Serial.println(K*(pTerm + iTerm + dTerm));
       
  
       return constrain(K*(pTerm + iTerm + dTerm), -127, 127);                     // Ausgabe des Motorwerts für die beiden Motoren innerhalb der Grenzen [-127,127]
      } 

 
 

// ******************************************************************************************************
// ************************************** Kalman filter module ******************************************
// ******************************************************************************************************


    float Q_angle  =  0.001;		// E(alpha2) = 0.001
    float Q_gyro   =  0.003;  		// E(bias2) = 0.003
    float R_angle  =  0.001;  		// Sz = 0.03   !!! je größer die Zahl, desto unempfindlicher reagiert der Winkel auf Veränderungen !!!
    float x_angle = 0;
    float x_bias = 0;
    float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
    float dt, y, S;
    float K_0, K_1;

  float kalmanCalculate(float newAngle, float newRate, int looptime)
     {
      dt = float(looptime)/1000;    // dt in Sekunden
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
// ******** Tastaturabfrage zur Veränderung der PUI-Parameter *********
// ********************************************************************

int Tastatureingabe()
   {
    if(!Serial.available())    return 0;
   
    char param = Serial.read();                            // get parameter byte
  
    if(!Serial.available())    return 0;
  
    char cmd = Serial.read();                              // get command byte
  
    Serial.flush();
  
    switch (param)
       {
        case 'p':
           if(cmd=='+')    Kp++;
           if(cmd=='-')    Kp--;
           break;
        case 'i':
           if(cmd=='+')    Ki += 0.1;
           if(cmd=='-')    Ki -= 0.1;
           break;
        case 'd':
           if(cmd=='+')    Kd++;
           if(cmd=='-')    Kd--;
           break;
       case 'k':
           if(cmd=='+')    K += 0.2;
           if(cmd=='-')    K -= 0.2;
           break;
       case 'l':
           if(cmd=='+')    K_Motor_links += 0.1;
           if(cmd=='-')    K_Motor_links -= 0.1;
           break;
       case 'r':
           if(cmd=='+')    K_Motor_rechts += 0.1;
           if(cmd=='-')    K_Motor_rechts -= 0.1;
           break;
     
       default:
           Serial.print("?");           Serial.print(param);
           Serial.print(" ?");          Serial.println(cmd);
      }
  
    Serial.println();
    Serial.print("K:");                      Serial.print(K);
    Serial.print("   Kp:");                  Serial.print(Kp);
    Serial.print("   Ki:");                  Serial.print(Ki);
    Serial.print("   Kd:");                  Serial.print(Kd);
    Serial.print("   K_Motor_links:");       Serial.print(K_Motor_links);
    Serial.print("   K_Motor_rechts:");      Serial.println(K_Motor_rechts);
   } 
