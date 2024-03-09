/*
  shashankmarch27@gmail.com

                     GNU GENERAL PUBLIC LICENSE
                        Version 3, 29 June 2007

  Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
  Everyone is permitted to copy and distribute verbatim copies
  of this license document, but changing it is not allowed.

  The GNU General Public License is a free, copyleft license for
  software and other kinds of works.

  THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY
  APPLICABLE LAW.  EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT
  HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY
  OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM
  IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF
  ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
*/

#include <Arduino.h>

#include<EEPROM.h>
#include<Adafruit_DPS310.h>
#include<Adafruit_BNO08x.h>

int sda_pin = 22;
int scl_pin = 19;

#if defined(BNO085)
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

struct telemetry_t {
  float packetCount;
  uint8_t mode;
  uint8_t state;
  float altitude;
  float temperature;
  float pressure;
  float voltgae;
  float gpsTime;
  float gpsLatitude;
  float gpsLongitude;
  uint8_t gpsSats;
  float tiltX;
  float tiltY;
  float rotZ;
}telemetry;

Adafruit_BNO08x  bno08x(BNO08X_RESET);

// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

#endif

#if defined(DPS310)
Adafruit_DPS310 baro;
#endif

// put function declarations here:
#if defined(BNO085)
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false);
void setReports(sh2_SensorId_t reportType, long report_interval);
#endif


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize serial communication

  Wire.setPins(sda_pin,scl_pin);

  #if defined(DPS310)
  if(!baro.begin_I2C()){
    Serial.println("Failed to find DPS310");
    while (1) yield();
  }
  baro.setMode(DPS310_CONT_PRESTEMP);
  baro.configurePressure(DPS310_1HZ, DPS310_64SAMPLES);
  baro.configureTemperature(DPS310_1HZ, DPS310_64SAMPLES);
  Serial.println("Initialized DPS310");
  #endif

  #if defined(BNO085)
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO085");
    while (1) yield();
  }
  Serial.println("Initialized BNO085");
  setReports(reportType, reportIntervalUs);
  #endif
  

}

void loop() {
  // put your main code here, to run repeatedly:

  #if defined(DPS310)
  sensors_event_t baro_temperature, baro_pressure;
  #endif

  #if defined(BNO085)
  sh2_SensorValue_t imuData;
  #endif

  if(baro.pressureAvailable() && baro.temperatureAvailable()){
    baro.getEvents(&baro_temperature,&baro_pressure);
    Serial.print(F("Temperature ="));
    Serial.print(baro_temperature.temperature);

    Serial.print(F("Pressure ="));
    Serial.print(baro_pressure.pressure);

    Serial.print(F("altitude ="));
    Serial.println(baro.readAltitude());
  }

  if (bno08x.wasReset()) {
    Serial.print("IMU was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&imuData) && imuData.sensorId == SH2_ARVR_STABILIZED_RV) {
    quaternionToEulerRV(&imuData.un.arvrStabilizedRV, &ypr, true);

    Serial.print(imuData.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(ypr.yaw);                Serial.print("\t");
    Serial.print(ypr.pitch);              Serial.print("\t");
    Serial.println(ypr.roll);
  }
}

// put function definitions here:

#if defined(BNO085)
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}
#endif