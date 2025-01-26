// //Alpha Rocket Avi Firmware Version 0.1.2

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h> //high accel
#include <Adafruit_LSM6DSO32.h> //low accel and gyro
#include <Adafruit_DPS310.h> //pressure

// //all thresholds are m/s^2
// #define G 9.80665
// #define TAKEOFF_THRESHOLD 20
// #define COAST_THRESHOLD 10
// #define DESCENT_THRESHOLD 0
// #define ABORT_THRESHOLD 10 //non-spinal accel

// float kalmanAlt();
// void deployChute();

// Adafruit_DPS310 baro;
// Adafruit_LSM6DSO32 gyro;
// Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

// File logfile;

// float initialAlt;

// int statusLED = 14;
// int flightPhase = 0; 
// int chipSelect = 17;
// /*0 = on pad
//   1 = powered flight
//   2 = coasting
//   3 = descent (hopefully this is when the parachute opens...)
//   4 = landed.*/

// int isChuteOpen = 0;

// void setup(void)
// {
//   Serial.begin(115200);
//   Serial.println("Begin init\n");

//   pinMode(statusLED, OUTPUT);
//   digitalWrite(statusLED, LOW);

//   /*hold init until all sensors have been found on the bus*/
//   while(!gyro.begin_I2C()){
//     Serial.println("Failed to find gyro\n");
//     delay(1000);
//   }
//   Serial.println("found gyro\n");

//   while(!accel.begin()){
//     Serial.println("Failed to find accelerometer\n");
//     delay(1000);
//   }
//   Serial.println("found accelerometer\n");

//   while(!baro.begin_I2C()){
//     Serial.println("Failed to find barometer\n");
//     delay(1000);
//   }
//   Serial.println("found barometer\n");

//   /*max it out baby!*/
//   gyro.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
//   gyro.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
//   gyro.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G); //options are 4, 8, 16, and 32.t
//   gyro.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

//   accel.setDataRate(ADXL343_DATARATE_3200_HZ);

//   baro.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
//   baro.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
//   initialAlt = baro.readAltitude();

//   // gyro.highPassFilter(true, 12); //might be needed

//   /*SD stuff*/
//   while(!SD.begin(chipSelect)){
//     Serial.println("Failed to find SD card\n");
//     delay(1000);
//   }
  
//   //begin the log
//   logfile = SD.open("log", FILE_WRITE);
//   logfile.println("Init success");
//   logfile.close();

//   Serial.println("\nInit success\n");
//   digitalWrite(statusLED, HIGH);

// }

// void loop(void)
// {
//   sensors_event_t linearaccel, angleaccel, temp, HA_accel, temp_event, pressure_event;

//   accel.getEvent(&HA_accel);
//   gyro.getEvent(&linearaccel, &angleaccel, &temp);
//   baro.getEvents(&temp_event, &pressure_event);

//   /*abort logic*/
//   if(flightPhase == 3 || (abs(HA_accel.acceleration.x) > ABORT_THRESHOLD || abs(HA_accel.acceleration.z) > ABORT_THRESHOLD) || (abs(linearaccel.acceleration.x) > ABORT_THRESHOLD || abs(linearaccel.acceleration.z) > ABORT_THRESHOLD)){
//     deployChute();
//   }

//   /*flight phase logic*/
//   if((linearaccel.acceleration.y > TAKEOFF_THRESHOLD && HA_accel.acceleration.y > TAKEOFF_THRESHOLD) && flightPhase == 0){flightPhase = 1;} //takeoff detection
//   if((linearaccel.acceleration.y > COAST_THRESHOLD && HA_accel.acceleration.y > COAST_THRESHOLD) && flightPhase == 1){flightPhase = 2;} //coast detection
//   if((linearaccel.acceleration.y > DESCENT_THRESHOLD && HA_accel.acceleration.y > DESCENT_THRESHOLD) && flightPhase == 2){flightPhase = 3;} //descent detection
//   //if(false){flightPhase = 4;} //landing detection

//   /*we do a bit of trig*/

//   // Serial.print(">X angle:");
//   // Serial.print(linearaccel.acceleration.);
//   // Serial.print("deg\n");

//   /*prints*/
//   Serial.print(">Flight Phase:");
//   Serial.print(flightPhase);
//   Serial.print("\n");

//   Serial.print(">Gyro Temp:");
//   Serial.print(temp.temperature);
//   Serial.print("deg C\n");

//   /* Display the results (acceleration is measured in m/s^2) */
//   Serial.print(">X Accel:");
//   Serial.print(linearaccel.acceleration.x);
//   Serial.print("m/s^2\n");

//   Serial.print(">Y Accel:");
//   Serial.print(linearaccel.acceleration.y);
//   Serial.print("m/s^2\n");

//   Serial.print(">Z Accel:");
//   Serial.print(linearaccel.acceleration.z);
//   Serial.print("m/s^2\n");

//   /* Display the results (rotation is measured in rad/s) */
//   Serial.print(">Gyro X:");
//   Serial.print(angleaccel.gyro.x);
//   Serial.print("radians/s\n");

//   Serial.print(">Gyro Y:");
//   Serial.print(angleaccel.gyro.y);
//   Serial.print("radians/s\n");

//   Serial.print(">Z:");
//   Serial.print(angleaccel.gyro.z);
//   Serial.print("radians/s\n");

//   /*high accel measurements*/
//   Serial.print(">HA X:"); 
//   Serial.print(HA_accel.acceleration.x);
//   Serial.print("m/s^2\n");

//   Serial.print(">HA Y:");
//   Serial.print(HA_accel.acceleration.y);
//   Serial.print("m/s^2\n");

//   Serial.print(">HA Z:");
//   Serial.print(HA_accel.acceleration.z);
//   Serial.print("m/s^2\n");

//   /*baro measurements*/
//   Serial.print(">Pressure:");
//   Serial.print(pressure_event.pressure);
//   Serial.print("hPa\n");

//   Serial.print(">Baro Temp:");
//   Serial.print(temp_event.temperature);
//   Serial.print("C\n");
  
//   delay(50); //don't forget to remove this
// }

// float kalmanAlt(){
//   //calculus here... eventually
//   return baro.readAltitude() - initialAlt;
// }

// void deployChute(){
//   isChuteOpen = 1;
//   digitalWrite(statusLED, LOW);

//   if(!isChuteOpen){ //if the chute is not open...
//     //servo logic here
//   }
// }
/*
  SD card test

  This example shows how use the utility libraries on which the
  SD library is based in order to get info about your SD card.
  Very useful for testing a card when you're not sure whether its working or not.
  Pin numbers reflect the default SPI pins for Uno and Nano models.
  The circuit:
    SD card attached to SPI bus as follows:
 ** SDO - pin 11 on Arduino Uno/Duemilanove/Diecimila
 ** SDI - pin 12 on Arduino Uno/Duemilanove/Diecimila
 ** CLK - pin 13 on Arduino Uno/Duemilanove/Diecimila
 ** CS - depends on your SD card shield or module.
 		Pin 10 used here for consistency with other Arduino examples

  created  28 Mar 2011
  by Limor Fried
  modified 24 July 2020
  by Tom Igoe
*/
// include the SD library:
#include <SPI.h>
#include <SD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Default SPI on Uno and Nano: pin 10
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKR Zero SD: SDCARD_SS_PIN
const int chipSelect = 17;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (1);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1 KB)
  Serial.print("Volume size (KB):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (MB):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (GB):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  root.close();
}

void loop(void) {
}