//Alpha Rocket Avi Firmware Version 0.1.1

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h> //high accel
#include <Adafruit_LSM6DSO32.h> //low accel and gyro
#include <Adafruit_DPS310.h> //pressure

#define G 9.80665

void deployChute();

Adafruit_DPS310 baro;
Adafruit_LSM6DSO32 gyro;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

int statusLED = 14;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Begin init\n");

  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);

  if(!gyro.begin_I2C()) {
    while(1){
      Serial.println("Failed to find gyro\n");
      delay(1000);
    }
  }

  if(!accel.begin())
  {
    while(1){
      Serial.println("Failed to find accelerometer\n");
      delay(1000);
    }
  }

  if(!baro.begin_I2C()){
      while(1){
      Serial.println("Failed to find barometer\n");
      delay(1000);
    }
  }

  //max it out baby!
  gyro.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  gyro.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
  gyro.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G); //options are 4, 8, 16, and 32.t 
  gyro.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  accel.setDataRate(ADXL343_DATARATE_3200_HZ);

  baro.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  baro.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  // gyro.highPassFilter(true, 12); //might be needed

  Serial.println("\nInit success\n");
  digitalWrite(statusLED, HIGH);

}

void loop(void)
{
  
  sensors_event_t linearaccel, angleaccel, temp, HA_accel, temp_event, pressure_event;

  accel.getEvent(&HA_accel);
  gyro.getEvent(&linearaccel, &angleaccel, &temp);
  baro.getEvents(&temp_event, &pressure_event);

  /*abort logic*/
  if(
    linearaccel.acceleration.x > 10 || linearaccel.acceleration.z > 10
  ){
    deployChute();
  }

  /*we do a bit of trig*/

  // Serial.print(">X angle:");
  // Serial.print(linearaccel.acceleration.);
  // Serial.print("deg\n");


  /*prints*/

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

  delay(50);
}

void deployChute(){

}