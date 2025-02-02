//Alpha Rocket Avi Firmware Version 0.2.0

#include <string.h> //yeah...
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h> //high accel
#include <Adafruit_LSM6DSO32.h> //low accel and gyro
#include <Adafruit_DPS310.h> //pressure

//all thresholds are m/s^2
#define G 9.80665
#define TAKEOFF_THRESHOLD 20
#define COAST_THRESHOLD 10
#define DESCENT_THRESHOLD 0
#define ABORT_THRESHOLD 10 //non-spinal accel

#define DEBUG 1 //1 means debug mode is on

float kalmanAlt();
void deployChute();

Adafruit_DPS310 baro;
Adafruit_LSM6DSO32 gyro;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

float initialAlt;

int buzzer = 15;
int statusLED = 14;
int chipSelect = 17;

int flightPhase = 0; 
/*0 = on pad
  1 = powered flight
  2 = coasting
  3 = descent (hopefully this is when the parachute opens...)
  4 = landed.*/

int isChuteOpen = 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Begin init\n");

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);

  /*hold init until all sensors have been found on the bus*/
  while(!gyro.begin_I2C()){
    Serial.println("Failed to find gyro\n");
    delay(1000);
  }
  Serial.println("found gyro\n");

  while(!accel.begin()){
    Serial.println("Failed to find accelerometer\n");
    delay(1000);
  }
  Serial.println("found accelerometer\n");

  while(!baro.begin_I2C()){
    Serial.println("Failed to find barometer\n");
    delay(1000);
  }
  Serial.println("found barometer\n");

  /*max it out baby!*/
  gyro.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  gyro.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  gyro.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G); //options are 4, 8, 16, and 32.t
  gyro.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  accel.setDataRate(ADXL343_DATARATE_3200_HZ);

  baro.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  baro.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  initialAlt = baro.readAltitude();

  // gyro.highPassFilter(true, 12); //might be needed

  /*SD stuff*/
  while(!SD.begin(chipSelect)){
    Serial.println("Failed to find SD card\n");
    delay(1000);
  }
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(" --- init success! --- ");
  dataFile.close();

  Serial.println("\nInit success\n");
  digitalWrite(statusLED, HIGH);

}
int count = 0;
int isOn = 1;
void loop(void)
{

  if(count > 50 ){count = 0; isOn = !isOn; if(isOn){digitalWrite(statusLED, LOW);}else{digitalWrite(statusLED, HIGH);}}
  count++;

  sensors_event_t linearaccel, angleaccel, temp, HA_accel, temp_event, pressure_event;

  accel.getEvent(&HA_accel);
  gyro.getEvent(&linearaccel, &angleaccel, &temp);
  baro.getEvents(&temp_event, &pressure_event);

  /*abort logic*/
  if(flightPhase == 3 || (abs(HA_accel.acceleration.x) > ABORT_THRESHOLD || abs(HA_accel.acceleration.z) > ABORT_THRESHOLD) || (abs(linearaccel.acceleration.x) > ABORT_THRESHOLD || abs(linearaccel.acceleration.z) > ABORT_THRESHOLD)){
    deployChute();
  }

  /*flight phase logic*/
  if((linearaccel.acceleration.y > TAKEOFF_THRESHOLD && HA_accel.acceleration.y > TAKEOFF_THRESHOLD) && flightPhase == 0){flightPhase = 1;} //takeoff detection
  if((linearaccel.acceleration.y > COAST_THRESHOLD && HA_accel.acceleration.y > COAST_THRESHOLD) && flightPhase == 1){flightPhase = 2;} //coast detection
  if((linearaccel.acceleration.y > DESCENT_THRESHOLD && HA_accel.acceleration.y > DESCENT_THRESHOLD) && flightPhase == 2){flightPhase = 3;} //descent detection
  //if(false){flightPhase = 4;} //landing detection


  /*logging, much room for improvement here*/
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  //timestamp
  dataFile.print(millis()); dataFile.print(","); 

  //gyro data
  dataFile.print(linearaccel.acceleration.x); dataFile.print(",");
  dataFile.print(linearaccel.acceleration.y); dataFile.print(",");
  dataFile.print(linearaccel.acceleration.z); dataFile.print(",");
  dataFile.print(angleaccel.acceleration.x); dataFile.print(",");
  dataFile.print(angleaccel.acceleration.y); dataFile.print(",");
  dataFile.print(angleaccel.acceleration.z); dataFile.print(",");

  //misc data
  dataFile.print(temp_event.temperature); dataFile.print(",");
  dataFile.print(pressure_event.pressure); dataFile.print(",");
  dataFile.println(flightPhase);

  dataFile.close();

  /*prints*/
  if(DEBUG){
  Serial.print(">Flight Phase:");
  Serial.print(flightPhase);
  Serial.print("\n");

  Serial.print(">Gyro Temp:");
  Serial.print(temp.temperature);
  Serial.print("deg C\n");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print(">X Accel:");
  Serial.print(linearaccel.acceleration.x);
  Serial.print("m/s^2\n");

  Serial.print(">Y Accel:");
  Serial.print(linearaccel.acceleration.y);
  Serial.print("m/s^2\n");

  Serial.print(">Z Accel:");
  Serial.print(linearaccel.acceleration.z);
  Serial.print("m/s^2\n");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print(">Gyro X:");
  Serial.print(angleaccel.gyro.x);
  Serial.print("radians/s\n");

  Serial.print(">Gyro Y:");
  Serial.print(angleaccel.gyro.y);
  Serial.print("radians/s\n");

  Serial.print(">Z:");
  Serial.print(angleaccel.gyro.z);
  Serial.print("radians/s\n");

  /*high accel measurements*/
  Serial.print(">HA X:"); 
  Serial.print(HA_accel.acceleration.x);
  Serial.print("m/s^2\n");

  Serial.print(">HA Y:");
  Serial.print(HA_accel.acceleration.y);
  Serial.print("m/s^2\n");

  Serial.print(">HA Z:");
  Serial.print(HA_accel.acceleration.z);
  Serial.print("m/s^2\n");

  /*baro measurements*/
  Serial.print(">Pressure:");
  Serial.print(pressure_event.pressure);
  Serial.print("hPa\n");

  Serial.print(">Baro Temp:");
  Serial.print(temp_event.temperature);
  Serial.print("C\n");
}

}

float kalmanAlt(){
  //calculus here... eventually
  return baro.readAltitude() - initialAlt;
}

void deployChute(){
  isChuteOpen = 1;
  //digitalWrite(statusLED, LOW);

  if(!isChuteOpen){ //if the chute is not open...
    //servo logic here
  }
}

//servo stuff:
//   The RP2040 PWM block has 8 identical slices. Each slice can drive two PWM output signals, or measure the frequency
//   or duty cycle of an input signal. This gives a total of up to 16 controllable PWM outputs. All 30 GPIO pins can be driven
//   by the PWM block
// *****************************************************************************************************************************/
// // This example to demo the new function setPWM_manual(uint8_t pin, uint16_t top, uint8_t div, uint16_t level, bool phaseCorrect = false)
// // used to generate a waveform. Check https://github.com/khoih-prog/RP2040_PWM/issues/6

// #define _PWM_LOGLEVEL_        2

// #if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
//       defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)

//   #if(_PWM_LOGLEVEL_>3)
//     #warning USING_MBED_RP2040_PWM
//   #endif

// #elif ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
//         defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)

//   #if(_PWM_LOGLEVEL_>3)
//     #warning USING_RP2040_PWM
//   #endif
// #else
//   #error This code is intended to run on the RP2040 mbed_nano, mbed_rp2040 or arduino-pico platform! Please check your Tools->Board setting.
// #endif

// #include "RP2040_PWM.h"

// #define UPDATE_INTERVAL       1000L

// // Using setPWM_DCPercentage_manual if true
// #define USING_DC_PERCENT      false   //true

// #define LED_ON        LOW
// #define LED_OFF       HIGH

// #define pinLed        25    // GP25, On-board BUILTIN_LED
// #define pin0          16    // GP16, PWM channel 4B (D2)
// #define pin10         10    // PWM channel 5A
// #define pin11         11    // PWM channel 5B

// #define pinToUse      pin10

// RP2040_PWM* PWM_Instance;

// float    frequency = 1000.0f;
// //float    frequency = 10000.0f;

// #if USING_DC_PERCENT
//   float    dutycyclePercent = 0.0f;
//   float    DCStepPercent    = 5.0f;
// #else
//   uint16_t dutycycle = 0;
//   uint16_t DCStep;
// #endif

// uint16_t PWMPeriod;

// char dashLine[] = "=================================================================================================";

// void printPWMInfo(RP2040_PWM* PWM_Instance)
// {
//   Serial.println(dashLine);
//   Serial.print("Actual data: pin = ");
//   Serial.print(PWM_Instance->getPin());
//   Serial.print(", PWM DutyCycle % = ");
//   Serial.print(PWM_Instance->getActualDutyCycle() / 1000.0f);
//   Serial.print(", PWMPeriod = ");
//   Serial.print(PWM_Instance->get_TOP());
//   Serial.print(", PWM Freq (Hz) = ");
//   Serial.println(PWM_Instance->getActualFreq(), 4);
//   Serial.println(dashLine);
// }

// void setup()
// {
//   Serial.begin(115200);

//   while (!Serial && millis() < 5000);

//   delay(100);

//   Serial.print(F("\nStarting PWM_manual on "));
//   Serial.println(BOARD_NAME);
//   Serial.println(RP2040_PWM_VERSION);

//   // Create a dummy instance
//   PWM_Instance = new RP2040_PWM(pinToUse, frequency, 0);

//   if (PWM_Instance)
//   {
//     uint16_t PWM_TOP   = PWM_Instance->get_TOP();
//     uint16_t PWM_DIV   = PWM_Instance->get_DIV();
//     uint16_t PWM_Level = 0;

//     // setPWM_manual(uint8_t pin, uint16_t top, uint8_t div, uint16_t level, bool phaseCorrect = false)
//     PWM_Instance->setPWM_manual(pinToUse, PWM_TOP, PWM_DIV, PWM_Level, true);
    
//     PWMPeriod = PWM_Instance->get_TOP();

// #if !USING_DC_PERCENT    
//     // 5% steps
//     DCStep = round(PWMPeriod / 20.0f);
// #endif
//   }
// }

// void loop()
// {
//   static unsigned long update_timeout = UPDATE_INTERVAL;

//   // Update DC every UPDATE_INTERVAL (100) milliseconds
//   if (millis() > update_timeout)
//   {
// #if USING_DC_PERCENT
//     PWM_Instance->setPWM_DCPercentage_manual(pinToUse, dutycyclePercent);

//     dutycyclePercent += DCStepPercent;

//     if (dutycyclePercent > 100.0f)
//       dutycyclePercent = 0.0f;
// #else
//     if (dutycycle > PWMPeriod)
//     {
//       //PWM_Instance->setPWM_manual(pinToUse, PWMPeriod);
//       PWM_Instance->setPWM_manual_Fast(pinToUse, PWMPeriod);
//       dutycycle = 0;
//     }
//     else
//     {
//       //PWM_Instance->setPWM_manual(pinToUse, dutycycle);
//       PWM_Instance->setPWM_manual_Fast(pinToUse, dutycycle);
//       dutycycle += DCStep;
//     }
// #endif

//     printPWMInfo(PWM_Instance);
    
//     update_timeout = millis() + UPDATE_INTERVAL;
//   }
// }