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
#include "RP2040_PWM.h" //pwm helper

//all thresholds are m/s^2
#define G 9.80665
#define TAKEOFF_THRESHOLD 20
#define COAST_THRESHOLD 10
#define DESCENT_THRESHOLD 1
#define ABORT_THRESHOLD 10 //non-spinal accel

#define _PWM_LOGLEVEL_ 3
#define SERVO_PIN 15
#define FREQ 200 //servo frequency in HZ
#define SERVO_OPEN_POS 34
#define SERVO_CLOSED_POS 10

#define BUZZER_PIN 3
#define STATUS_LED_PIN 14
#define SD_CS_PIN 17 //SD module chip select

#define DEBUG 1 //1 means debug mode is ON

void updateState(float, float, float, float);
float kalmanAlt();
void deployChute();

Adafruit_DPS310 baro;
Adafruit_LSM6DSO32 gyro;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
RP2040_PWM* ServoPWM;
RP2040_PWM* BuzzerPWM;

typedef struct vehicleState{ //i just learned how to do this lol
  float pos[3];
  float vel[3];
  float accel[3];
};

vehicleState currentState;

float initialAlt;

int flightPhase = 2;
/*0 = on pad
  1 = powered flight
  2 = coasting
  3 = descent (hopefully this is when the parachute opens...)
  4 = landed.*/

unsigned long current = 0;
unsigned long last = 0;
int chuteTimeout = 0;
int triggerCount = 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Begin init\n");

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  ServoPWM = new RP2040_PWM(SERVO_PIN, FREQ, SERVO_CLOSED_POS);
  BuzzerPWM = new RP2040_PWM(BUZZER_PIN, 1, 0);

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
  while(!SD.begin(SD_CS_PIN)){
    Serial.println("Failed to find SD card\n");
    delay(1000);
  }
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(" --- init success! --- ");
  dataFile.close();

  Serial.println("\nInit success\n");

  //startup sounds are fun
  BuzzerPWM->setPWM(BUZZER_PIN, 750, 50);
  delay(100);
  BuzzerPWM->setPWM(BUZZER_PIN, 950, 50);
  delay(100);
  BuzzerPWM->setPWM(BUZZER_PIN, 1150, 50);
  delay(400);
  BuzzerPWM->setPWM(BUZZER_PIN, 1150, 0);

  //set servo to closed position (pedantic)
  ServoPWM->setPWM(SERVO_PIN, FREQ, SERVO_CLOSED_POS);

  digitalWrite(STATUS_LED_PIN, HIGH);

}
int count = 0;
int isOn = 1;
void loop(void)
{


  if(count > 50 ){count = 0; isOn = !isOn; if(isOn){digitalWrite(STATUS_LED_PIN, LOW); BuzzerPWM->setPWM(BUZZER_PIN, 900, 0);}else{digitalWrite(STATUS_LED_PIN, HIGH); BuzzerPWM->setPWM(BUZZER_PIN, 1150, 0);}}
  count++;

  sensors_event_t linearaccel, angleaccel, temp, HA_accel, temp_event, pressure_event;

  accel.getEvent(&HA_accel);
  gyro.getEvent(&linearaccel, &angleaccel, &temp);
  baro.getEvents(&temp_event, &pressure_event);

  /*abort logic*/
  if(flightPhase == 3){ //if(flightPhase == 3 || (abs(HA_accel.acceleration.x) > ABORT_THRESHOLD || abs(HA_accel.acceleration.z) > ABORT_THRESHOLD) || (abs(linearaccel.acceleration.x) > ABORT_THRESHOLD || abs(linearaccel.acceleration.z) > ABORT_THRESHOLD)){
    deployChute();
  }

  /*flight phase logic*/
  if((linearaccel.acceleration.y > TAKEOFF_THRESHOLD && HA_accel.acceleration.y > TAKEOFF_THRESHOLD) && flightPhase == 0){flightPhase = 1;} //takeoff detection
  if((linearaccel.acceleration.y > COAST_THRESHOLD && HA_accel.acceleration.y > COAST_THRESHOLD) && flightPhase == 1){flightPhase = 2;} //coast detection
  if((linearaccel.acceleration.y < DESCENT_THRESHOLD && HA_accel.acceleration.y < DESCENT_THRESHOLD) && flightPhase == 2){flightPhase = 3;} //descent detection
  //if(false){flightPhase = 4;} //landing detection

  // current = micros();
  // updateState(linearaccel.acceleration.x, linearaccel.acceleration.y, linearaccel.acceleration.z, (current-last));
  // last = micros();

  /*logging, much room for improvement here*/
  if(!DEBUG){

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
  }

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

  // Serial.print(">Estimated X pos:");
  // Serial.print(currentState.pos[0]);
  // Serial.print("M\n");

  // Serial.print(">Estimated Y pos:");
  // Serial.print(currentState.pos[1]);
  // Serial.print("M\n");

  // Serial.print(">Estimated Z pos:");
  // Serial.print(currentState.pos[2]);
  // Serial.print("M\n");

}

}

void updateState(float accelx, float accely, float accelz, float dt){

  dt = dt / 1000000; //convert microseconds to seconds.

  currentState.accel[0] = accelx;
  currentState.accel[1] = accely;
  currentState.accel[2] = accelz;

  currentState.vel[0] += currentState.accel[0] * dt;
  currentState.vel[1] += (currentState.accel[1] - G) * dt;
  currentState.vel[2] += currentState.accel[2] * dt;

  currentState.pos[0] += currentState.vel[0] * dt;
  currentState.pos[1] += currentState.vel[1] * dt;
  currentState.pos[2] += currentState.vel[2] * dt;
}

float kalmanAlt(){
  int baroAlt = baro.readAltitude() - initialAlt;
  return 0.0;
}

void deployChute(){
  if(triggerCount < 4){ //don't toggle the servo more than 4 times, end with servo in closed position to reduce strain
    if(chuteTimeout < 50){
      ServoPWM->setPWM(SERVO_PIN, FREQ, SERVO_OPEN_POS);
    }else if(chuteTimeout > 50 && chuteTimeout != 100){
      ServoPWM->setPWM(SERVO_PIN, FREQ, SERVO_CLOSED_POS);
    }else if(chuteTimeout == 100){
      chuteTimeout = 0;
      triggerCount++;
    }
    chuteTimeout++;
  }
}
