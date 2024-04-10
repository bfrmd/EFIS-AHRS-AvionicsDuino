/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                                 AHRS_Avionicsduino Version 2.1
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  AHRS_Avionicsduino V 2.1 is free software    
  MIT License (MIT)

  Copyright (c) 2024 AvionicsDuino - benjamin.fremond@avionicsduino.com
  https://avionicsduino.com/index.php/en/the-ahrs/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
    
Cet AHRS est basé sur une carte Teensy 4.0, un module IMU 9 DOF Adafruit ADA4517 (LSM6DSOX & LIS3MDL), un GNSS NEO M9N, et le logiciel de fusion UNavINS de Bolder Flight Systems (15 State Extended Kalman Filter).

Les éléments transmis au CAN Bus sont les suivants (l'AHRS ne filtre aucune valeur avant envoi au CAN bus):

- float       roll                Angle de roulis en radians
- float       pitch               Angle de tangage en degrés
- float       altitudeAMSL_GNSS   Altitude AMSL en pieds
- float       groundSpeedGNSS     Vitesse sol en km/h
- float       accY                Accélération Y en m/s² 
- float       accZ                Accélération Z en m/s² 
- float       vz                  Vitesse verticale en m/s (valeur transmise positive vers le haut)
- float       trk                 Route GNSS en degrés, de -180° à +180°
- uint16_t    yearGNSS            Année
- uint8_t     monthGNSS           Mois
- uint8_t     dayGNSS             Jour 
- uint8_t     hourGNSS            Heure
- uint8_t     minuteGNSS          Minutes
- uint8_t     secondGNSS          Secondes
- int32_t     nanoSecGNSS         Nanosecondes
- uint32_t    timeAccuracyGNSS    Nanosecondes
- uint8_t     satInView           Nombre de satellites GNSS en vue
- double      latitude            latitude en degrés
- double      longitude           longitude en degrés
- uint64_t    inc                 Valeur incrémentée d'une unité après chaque envoi au CAN (dans un but de contrôle de l'absence de perte de donnée par le FDR)
*/

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques des différents composants avec la carte Teensy 4.0
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Le module IMU ADAFRUIT ADA-4517 est connecté en I2C, il est exploité par les bibliothèques Adafruit LSM6DSOX & LIS3MDL
//---------------ADA4517------------------------Teensy 4.0------------------------------------------------------------------------------------------------------
//                GND     ------------------->   GND
//                Vin    -------------------->   + 3.3 volts
//                SDA   --------------------->   SDA1 (pin 17)  résistance pull sur la carte Adafruit 
//                SCL   --------------------->   SCL1 (pin 16)  résistance pull sur la carte Adafruit

//--------------------------------------------------------------------------------------------------------------------------------------------------------
// Le GPS (SparkFun GPS Breakout - NEO-M9N, U.FL) est connecté en Serial3 comme ci-dessous, il est exploité par la bibliothèque ubx Bolder Flight Systems
// -------------NEO-M9N------------------------Teensy 4.0-------------------------------------------------------------------------------------------------
//                GND     ------------------->   GND
//             Alim + 5 v ------------------->   + 5 volts
//                Rx ----------------------->   TX3 (pin 14)
//                Tx ----------------------->   RX3 (pin 15)
//
//---------------------------------------------------------------------------------------------------------------------------------------------------------
//            Connecteur du module AHRS vers l'EFIS hôte
//------------ Connecteur AHRS --------------- Teensy 4.0 -------------------------------------------------------------------------------------------------
//                 Pin 1 GND  ----------------> GND
//                 Pin 2   -------------------> CAN H
//                 Pin 3 alim 5V -------------> + 5 volts
//                 Pin 4 Rx ------------------> La transmission série n'est pas utilisée dans cette version du programme
//                 Pin 5 Tx ------------------> La transmission série n'est pas utilisée dans cette version du programme
//                 Pin 6 ---------------------> CAN L

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques et fichiers externes
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <uNavINS.h> 
#include <ubx.h> 
#include <Adafruit_LSM6DSOX.h> 
#include <Adafruit_LIS3MDL.h>
#include <TeensyTimerTool.h>
using namespace TeensyTimerTool; 
#include <FlexCAN_T4.h>

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

bfs::Ubx Gnss(&Serial3); 
uNavINS Filter; 
Adafruit_LSM6DSOX lsm6ds; 
Adafruit_LIS3MDL lis3mdl; 
PeriodicTimer TimerSensorData; 
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CAN_Module_AHRS;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ************************************************************** Variables IMU ***************************************************************
#define pinINT2 13 
bool OKFilter=false; 
bool OKGetSensorData = false; 
sensors_event_t accel, gyro, mag, temp; 

// ************************************************************** Variables horizon ***************************************************************
float roll, pitch, accY, accZ, vz, trk, satInView, altitudeAMSL_GNSS, groundSpeedGNSS; 
double latitude, longitude;

// ************************************************************** Variables date et heure ***************************************************************
uint16_t yearGNSS;
uint8_t monthGNSS;
uint8_t dayGNSS;
uint8_t hourGNSS;
uint8_t minuteGNSS;
uint8_t secondGNSS; 
uint32_t nanoSecGNSS;
uint32_t timeAccuracyGNSS;

// ************************************************************** Variables CAN Bus ***************************************************************
CAN_message_t msgRollPitch;
CAN_message_t msgAccYAccZ;
CAN_message_t msgVzTrk;
CAN_message_t msgDateTimeSat;
CAN_message_t msgAltiSpeedGNSS;
CAN_message_t msgLatitude;
CAN_message_t msgLongitude;
CAN_message_t msgNanoSecTimeAccur;
CAN_message_t msgTest; // Message de débogage pour vérifier l'absence de perte de données
uint64_t inc = 0;      // Variable transmise au CAN dans le message msgTest, puis incrémentée

// ************************************************ Variables utilisées pour les envois périodiques de données au CAN Bus **********************************
uint32_t periode50msStartTime;      // Variable utilisée pour les envois à 20 Hz
uint16_t periode50msTimeOut = 50;
uint32_t periode100msStartTime;      // Variable utilisée pour les envois à 10 Hz
uint16_t periode100msTimeOut = 100;
uint32_t periode200msStartTime;      // Variable utilisée pour les envois à 5 Hz
uint16_t periode200msTimeOut = 200;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                        SETUP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  pinMode(9, OUTPUT); // pour le débogage à l'oscilloscope
  
// ********************************************************** Initialisation des messages CAN Bus ****************************************************************************
  msgRollPitch.id = 20;
  msgRollPitch.len = 8;
  msgAccYAccZ.id = 22;
  msgAccYAccZ.len = 8;
  msgVzTrk.id = 24;
  msgVzTrk.len = 8;
  msgDateTimeSat.id = 34;
  msgDateTimeSat.len = 8;
  msgNanoSecTimeAccur.id = 38;
  msgNanoSecTimeAccur.len = 8;
  msgAltiSpeedGNSS.id = 30;
  msgAltiSpeedGNSS.len = 8;
  msgLatitude.id = 36;
  msgLatitude.len = 8;
  msgLongitude.id = 37;
  msgLongitude.len = 8;
  
  msgTest.id = 10;
  msgTest.len = 8;


// ********************************************************** Initialisation des voies série USB et UART3  ****************************************************************************
  Serial.begin(115200);
  Serial3.begin(921600); // pour la communication avec le GNSS
//********************************************************** Initialisation du timer ********************************************************************
  TimerSensorData.begin(SetOKGetSensorData, 20ms); 
  pinMode(pinINT2, INPUT_PULLUP); 
  attachInterrupt(pinINT2, SetOKFilter, RISING); 
  
// ******************************************************************* Initialisation du CAN bus  **************************************************************************
  CAN_Module_AHRS.begin();
  CAN_Module_AHRS.setBaudRate(500000);
  CAN_Module_AHRS.setMaxMB(16);
  CAN_Module_AHRS.enableFIFO();
  CAN_Module_AHRS.enableFIFOInterrupt();
  CAN_Module_AHRS.mailboxStatus();
  delay(155);

// ******************************************************************** Initialisation du GNSS U-BLOX NEO M9N  ***************************************************************
  if (!Gnss.Begin(921600)) 
  {
    Serial.println("Impossible de communiquer avec le GNSS");
    while (1) {}
  }
  delay(155);
  Serial.println("Communication établie avec le GNSS");

// ************************************************************* Initialisation du module IMU Adafruit *****************************************************************
  Serial.println("Adafruit LSM6DS+LIS3MDL test..... ");
  bool lsm6ds_success, lis3mdl_success;
  lsm6ds_success = lsm6ds.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire1);
  lis3mdl_success = lis3mdl.begin_I2C(LIS3MDL_I2CADDR_DEFAULT,&Wire1);
  if (!lsm6ds_success){ Serial.println("Failed to find LSM6DS chip");}
  if (!lis3mdl_success){Serial.println("Failed to find LIS3MDL chip");}
  if (!(lsm6ds_success && lis3mdl_success)) {while (1) {delay(10);}} 
  Serial.println("LSM6DS and LIS3MDL Found!");
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_52_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_52_HZ);
  lsm6ds.configIntOutputs(true, false);
  lsm6ds.configInt2(false, true, true);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  delay(155); 
  
// ****************************************** Initialisation de quelques valeurs particulières constantes ou provisoire de tabTrame ********************************************
pitch = 90.0; // On introduit un angle de pitch positif de 90°, de façon à ne voir que du bleu sur l'EFIS avant que le GNSS n'ait fait son fix.
periode50msStartTime = millis();
periode100msStartTime = millis();
periode200msStartTime = millis();

} // ********************************************************************* Fin du setup **************************************************************************************

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                           LOOP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{ 
// *********************************** Recherche de données IMU nouvelles, application du filtre EKF et mise à jour des variables *******************************
  Gnss.Read(); 
  if (Gnss.num_sv() > 5)
  {
    if (OKGetSensorData) // Condition vraie par timer toutes les 20 ms (soit 50 Hz), afin d'autoriser la lecture des données IMU et l'application du filtre EKF.
    {
      OKGetSensorData = false;
      lsm6ds.getEvent(&accel, &gyro, &temp); 
      lis3mdl.getEvent(&mag);  
    }
    if (OKFilter) // Condition vraie s'il y a eu une interruption externe déclenchée par la carte IMU, donc si des données nouvelles étaient effectivement prêtes,
                  // auquel cas le filtre EKF peut être appliqué, et les variables peuvent être mises à jour.
    {
      OKFilter=false;          
      Filter.update(Gnss.gps_tow_s(),
                    Gnss.north_vel_mps(),
                    Gnss.east_vel_mps(),
                    Gnss.down_vel_mps(),
                    Gnss.lat_rad(),Gnss.lon_rad(),
                    Gnss.alt_msl_m(),
                    gyro.gyro.x,
                    -gyro.gyro.y,           // Certaines valeurs sont inversées, car les axes du module IMU ADA 4517 ne correspondent pas aux axes attendus par le filtre EKF
                    -gyro.gyro.z,           // Les axes attendus par le filtre sont ceux de l'aéronautique en général : X positif vers l'avant, Y positif vers la droite, Z positif vers le bas
                    accel.acceleration.x,   // Alors que les axes du module ADA4517 sont : X positif vers l'avant, Y positif vers la gauche, Z positif vers le haut
                    -accel.acceleration.y,  // Les axes Y et Z doivent donc être inversés.
                    -accel.acceleration.z,
                    mag.magnetic.x,
                    -mag.magnetic.y,
                    -mag.magnetic.z);
      
      pitch = Filter.getPitch_rad()* (180 / PI); // L'AHRS transmet au CAN Bus un angle de pitch non filtré, en degrés.
      roll = Filter.getRoll_rad(); // L'AHRS transmet au CAN Bus un angle de roll non filtré, en radians.
      accY = -accel.acceleration.y; // Valeur transmise au CAN bus en m/s². On ne filtre pas accY. Cette valeur est filtrée au niveau de l'EFIS
      accZ = -accel.acceleration.z; // Valeur transmise au CAN bus en m/s². On ne filtre pas accZ, pour pouvoir saisir des accélérations importantes mais brèves (utile pour l'EFIS pour calculer les G min et max encaissés).   
      vz = Filter.getVelDown_ms(); // Le filtre EKF 15 states fournit la valeur de vz, en m/s (valeur transmise positive vers le haut)
      trk = Filter.getGroundTrack_rad()*(180/PI); //Le filtre EKF fournit la valeur de trk en radians, on la convertit en degrés, de -180° à +180°.
      
      altitudeAMSL_GNSS = Gnss.alt_msl_m();             // Altitude AMSL passée en mètres par le GNSS
      altitudeAMSL_GNSS = altitudeAMSL_GNSS * 3.28084;  // conversion en pieds pour transmission sans filtrage au CAN bus

      groundSpeedGNSS = Gnss.gnd_spd_mps();            //Vitesse sol passée en m/s par le GNSS
      groundSpeedGNSS = groundSpeedGNSS * 3.6;         // Conversion en km/h pour transmission sans filtrage au CAN bus
      
      yearGNSS = Gnss.utc_year();
      monthGNSS = Gnss.utc_month();
      dayGNSS = Gnss.utc_day();
      hourGNSS = Gnss.utc_hour();
      minuteGNSS = Gnss.utc_min();
      secondGNSS = Gnss.utc_sec();
      nanoSecGNSS = Gnss.utc_nano();
      timeAccuracyGNSS = Gnss.time_acc_ns();
      satInView = Gnss.num_sv();
      
      latitude = Filter.getLatitude_rad();    // latitude passée par le filtre EKF en radians.
      latitude = latitude * (180 / PI);       // latitude convertie dégrés pour transmission au CAN bus.
      longitude = Filter.getLongitude_rad();  // longitude passée par le filtre EKF en radians.
      longitude = longitude * (180 / PI);     // latitude convertie dégrés pour transmission au CAN bus.

    }
  //Serial.printf("Latitude en degrés :  %2.10f %s",latitude, "\n");
  //Serial.printf("Longitude en degrés :  %2.10f %s",longitude, "\n");  
  //Serial.printf("Nombre de satellites : %2.0f   %04d/%02d/%02d  %02d:%02d:%02d  nanoSec : %012d   %s  accuracy : %012u   %s  ",satInView,yearGNSS,monthGNSS,dayGNSS,hourGNSS,minuteGNSS,secondGNSS,nanoSecGNSS," ns ",timeAccuracyGNSS," ns\n" );
  //Serial.printf("Pitch en degrés :  %2.0f %s",pitch, "\n");
  //Serial.printf("Roll en degrés :  %2.0f %s",roll * (180 / PI), "\n");
  //Serial.println("--------------------------------------------------"); 
  
  } 
// ********************************************************************** Envoi périodique des données sur le CAN bus *************************************************************** 

  // +++++++++++++++ Envois à 20 Hz  +++++++++++++++++++++++++++++++++++++++++
  if ((millis()- periode50msStartTime)>= periode50msTimeOut) 
  {
    periode50msStartTime = millis();
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgRollPitch.buf[i] = ((byte*) &roll)[i];
      msgAccYAccZ.buf[i]  = ((byte*) &accY)[i];
      msgRollPitch.buf[i+4] = ((byte*) &pitch)[i];
      msgAccYAccZ.buf[i+4]  = ((byte*) &accZ)[i];
    }
    for (uint8_t i = 0; i < 8; i++ )
    {
      msgTest.buf[i] = ((byte*) &inc)[i];
    }
    inc++;
    CAN_Module_AHRS.write(msgTest);   
    CAN_Module_AHRS.write(msgRollPitch);
    CAN_Module_AHRS.write(msgAccYAccZ);
  }

  // +++++++++++++++ Envois à 10 Hz  +++++++++++++++++++++++++++++++++++++++++
  if ((millis()- periode100msStartTime)>= periode100msTimeOut) 
  {
    periode100msStartTime = millis();
    for (uint8_t i = 0; i < 4; i++ )
    {
      msgVzTrk.buf[i]   = ((byte*) &vz )[i];
      msgVzTrk.buf[i+4] = ((byte*) &trk)[i];
      msgAltiSpeedGNSS.buf[i]   = ((byte*) &altitudeAMSL_GNSS)[i];
      msgAltiSpeedGNSS.buf[i+4] = ((byte*) &groundSpeedGNSS)[i];
    }
    
    CAN_Module_AHRS.write(msgVzTrk);
    CAN_Module_AHRS.write(msgAltiSpeedGNSS);
    
  }

  // +++++++++++++++ Envois à 5 Hz  +++++++++++++++++++++++++++++++++++++++++
  if ((millis()- periode200msStartTime)>= periode200msTimeOut)
  {
    periode200msStartTime = millis();
    msgDateTimeSat.buf[0] = ((byte*) &yearGNSS)[0];
    msgDateTimeSat.buf[1] = ((byte*) &yearGNSS)[1];
    msgDateTimeSat.buf[2] = monthGNSS;
    msgDateTimeSat.buf[3] = dayGNSS;
    msgDateTimeSat.buf[4] = hourGNSS;
    msgDateTimeSat.buf[5] = minuteGNSS;
    msgDateTimeSat.buf[6] = secondGNSS;
    msgDateTimeSat.buf[7] = satInView;
   
   for (uint8_t i = 0; i < 4; i++ )
    {
      msgNanoSecTimeAccur.buf[i]  = ((byte*) &nanoSecGNSS)[i];
      msgNanoSecTimeAccur.buf[i+4]  = ((byte*) &timeAccuracyGNSS)[i];
    }
    
    for (uint8_t i = 0; i < 8; i++ )
    {
      msgLatitude.buf[i] = ((byte*) &latitude)[i];
      msgLongitude.buf[i] = ((byte*) &longitude)[i];
    }
    CAN_Module_AHRS.write(msgDateTimeSat);
    CAN_Module_AHRS.write(msgNanoSecTimeAccur);
    CAN_Module_AHRS.write(msgLatitude);
    CAN_Module_AHRS.write(msgLongitude);
  } 
} //************************************************************************* Fin de la boucle loop() ************************************************************************


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   ISR de timing
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SetOKGetSensorData()
{
  //digitalToggleFast(9); // pour débogage à l'oscilloscope
  OKGetSensorData = true;
}

void SetOKFilter()
{
  OKFilter=true;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonction utilisée pour le filtrage des données brutes issues des capteurs
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Fonction de filtre à Réponse Impulsionnelle Infinie (RII)
float iirFilter (float previousFilteredValue, float currentValue , float iirFilterCoefficient)
{
  return previousFilteredValue  * (1 - iirFilterCoefficient) + currentValue * iirFilterCoefficient ;
}
