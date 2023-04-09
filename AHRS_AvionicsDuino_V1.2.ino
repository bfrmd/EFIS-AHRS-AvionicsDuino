/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                                 AHRS_Avionicsduino Version 1.2
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  AHRS_Avionicsduino_V1.2 is free software    
  MIT License (MIT)
  
  Copyright (c) 2023 AvionicsDuino - benjamin.fremond@avionicsduino.com
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
    
 *****************************************************************************************************************************/ 

//*********************************************************** AHRS Teensy 4.0 / LSM6DSOX+LIS3MDL / NEOM9N / UnavINS **************************************************************
// Cet AHRS est basé sur une Teensy 4.0, un module IMU 9 DOF Adafruit ADA4517 (LSM6DSOX & LIS3MDL), un GNSS NEO M9N, et le logiciel de fusion UNavINS de BolderFlight (15 State Extended Kalman Filter).
// La bibliothèque UNavIns est téléchargeable ici : https://forum.pjrc.com/threads/48856-uNav-INS/page31/post#759
// Les dépendances (Eigen et Units) sont disponibles ici : https://github.com/bolderflight/eigen et https://github.com/bolderflight/units
// La bibliothèque ubx est disponible ici : https://github.com/bolderflight/ublox
// La vitesse du port série (en sortie vers l'EFIS hôte)est 115200 bds. Par défaut, le module n'envoie rien, il faut que l'EFIS lui envoyer le caractère 'F' pour qu'il réponde par une trame
// dont la structure est indiquée ci-dessous.
/*
La trame de sortie est la suivante :
- Byte('F');           Offset 00  Caractère “F” majuscule indiquant le début de trame      
- Byte(18);            Offset 01  Octet constant de valeur 18 
- Float(Phi);          Offset 02  Angle de roulis en radians
- Float(The);          Offset 06  Angle de tangage en radians
- Float(Psi);          Offset 10  Angle de lacet en radians 
- Float(AltMSL)        Offset 14  Altitude MSL en mètres
- Float(gndSpeed)      Offset 18  GroundSpeed en m/s
- uint16_t(year)       Offset 22  Année
- uint8_t(month)       Offset 24  Mois
- uint8_t(day)         Offset 25  Jour 
- uint8_t(hour)        Offset 26  Heure
- uint8_t(min)         Offset 27  Minutes
- uint8_t(sec)         Offset 28  Secondes

                       Les octets d'offset [29 - 45] sont disponibles pour de futures évolutions

- Float(AccY);         Offset 46  Accélération Y en m/s² 
- Float(AccZ);         Offset 50  Accélération Z en m/s² 
- Float(-VitD);        Offset 54  Vitesse verticale en m/s (valeur transmise positive vers le haut)
- Float(-1);           Offset 58  Constante égale à -1
- Float(CapGps);       Offset 62  Cap GPS (TK, ou Track) en radians 

                       Les octets d'offset [66 - 69] sont disponibles pour de futures évolutions

- Float(GpsNumSV);     Offset 70  Nombre de satellites GPS utilisés

*/

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques des différents composants avec la carte Teensy 4.0
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
// Le module IMU ADAFRUIT ADA-4517 est connecté en I2C 1, il est exploité par les bibliothèques Adafruit LSM6DSOX & LIS3MDL
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
//                 Pin 4 Rx ------------------> RX2 (pin 9)
//                 Pin 5 Tx ------------------> TX2 (pin 10)
//                 Pin 6 ---------------------> CAN L

// Le PCB comporte un emplacement pour un transceiver CAN BUS MCP2562 EP facultatif qui n'est pas exploité dans la version actuelle de ce sketch. 
// Il est prévu pour l'avenir que l'AHRS transmette ses données à l'EFIS plutôt via le bus CAN que via la voie série.

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques et fichiers externes
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "uNavINS.h" 
#include "ubx.h" 
#include <Adafruit_LSM6DSOX.h> // Bibliothèques Adafruit pour le module IMU ADA 4517
#include <Adafruit_LIS3MDL.h>
#include <TeensyTimerTool.h>
using namespace TeensyTimerTool; 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

bfs::Ubx Gnss(&Serial3); 
uNavINS Filter; 
Adafruit_LSM6DSOX lsm6ds; 
Adafruit_LIS3MDL lis3mdl; 
PeriodicTimer TimerSensorData; 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#define pinINT2 13 
bool OKFilter=false; 
bool OKGetSensorData=false; 
sensors_event_t accel, gyro, mag, temp; 

// ************************************************************** Variables horizon ***************************************************************
float heading;
byte tabTrame[74]; // Tableau prévu pour contenir une trame complète de 74 octets. 
float roll=0, pitch=0, psi=0, AccX, AccY, AccZ, Vz, constante=-1, trk, nbSat; 
char a; byte b; 
float altitudeMslGNSS;
float groundSpeedGNSS;
uint16_t anneeGNSS;
uint8_t moisGNSS;
uint8_t jourGNSS;
uint8_t heureGNSS;
uint8_t minuteGNSS;
uint8_t secondeGNSS; 
 

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           SETUP
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
// ************************************************************** Initialisation des voies série  ****************************************************************************
  Serial.begin(115200);
  Serial2.begin(115200); // Cette voie série est destinée à la communication avec l'EFIS hôte

//****************************************************************** Initialisation des interruptions, timers et broches de surveillance oscilloscopique
  pinMode(pinINT2, INPUT_PULLUP); 
  attachInterrupt(pinINT2, SetOKFilter, RISING); 
  TimerSensorData.begin(SetOKGetSensorData, 20ms); 

// ******************************************************************** Initialisation du GNSS U-BLOX NEO M9N  ***************************************************************
  if (!Gnss.Begin(921600)) 
  {
    Serial.println("Impossible de communiquer avec le GNSS");
    while (1) {}
  }
  delay(500);
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
  
//****************************************************** Initialisation à zéro de tous les octets du buffer d'envoi des trames *********************************************************
  for(uint8_t n = 0; n<74; n++)
      {
        tabTrame[n]=0;
      }
// *************************************************** puis initialisation de quelques valeurs particulières constantes ou provisoire de tabTrame *******************************************************
  tabTrame[0]=byte('F');
  tabTrame[1]=18;
  for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+58] = ((byte*) &constante)[i];
  float piDivTwo = PI/2;
  for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+6] = ((byte*) &piDivTwo)[i]; // On introduit un angle de pitch positif de 90°, de façon à ne voir que du bleu sur l'EFIS avant que le GNSS n'ait fait son fix.
  
  delay(500); 
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                             LOOP
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{ 
  Gnss.Read(); 
  if (Gnss.num_sv() > 5) 
  {
    if (OKGetSensorData) 
    {
      OKGetSensorData = false;
      lsm6ds.getEvent(&accel, &gyro, &temp); 
      lis3mdl.getEvent(&mag);                
    }
    if (OKFilter) 
    {
      OKFilter=false; 
      digitalWrite(0, !digitalRead(0)); 
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
      

      pitch = Filter.getPitch_rad();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+6] = ((byte*) &pitch)[i];
      
      roll = Filter.getRoll_rad();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+2] = ((byte*) &roll)[i];
  
      heading = Filter.getHeading_rad();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+10] = ((byte*) &heading)[i];
  
      Vz = Filter.getVelDown_ms();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+54] = ((byte*) &Vz)[i];
  
      trk = Filter.getGroundTrack_rad();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+62] = ((byte*) &trk)[i];
  
      nbSat = Gnss.num_sv();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+70] = ((byte*) &nbSat)[i];
  
      AccY = -accel.acceleration.y; 
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+46] = ((byte*) &AccY)[i];

      AccZ = -accel.acceleration.z; 
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+50] = ((byte*) &AccZ)[i];

      altitudeMslGNSS = Gnss.alt_msl_m();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+14] = ((byte*) &altitudeMslGNSS)[i];
      
      groundSpeedGNSS = Gnss.gnd_spd_mps();
      for (uint8_t i = 0; i < 4; i++ ) tabTrame[i+18] = ((byte*) &groundSpeedGNSS)[i];
      
      anneeGNSS = Gnss.utc_year();
      for (uint8_t i = 0; i < 2; i++ ) tabTrame[i+22] = ((byte*) &anneeGNSS)[i];
      
      moisGNSS = Gnss.utc_month();
      tabTrame[24]=moisGNSS;
      
      jourGNSS = Gnss.utc_day();
      tabTrame[25]=jourGNSS;
      
      heureGNSS = Gnss.utc_hour();
      tabTrame[26]=heureGNSS;
      
      minuteGNSS = Gnss.utc_min();
      tabTrame[27]=minuteGNSS;
      
      secondeGNSS = Gnss.utc_sec();
      tabTrame[28]=secondeGNSS; 
      
    }
  }  
}
//***************************************************************************************************** Fin de la boucle infinie Loop ***********************************************************************************************


void serialEvent2() 
{
  
  if(char(Serial2.read()=='F'))
  {
    for(uint8_t n = 0; n<74; n++)
    {
      Serial2.write(tabTrame[n]);
    }
  }
}

void SetOKFilter()
{
  OKFilter=true;
}

void SetOKGetSensorData()
{
  OKGetSensorData = true;
}
