/**
 * @file main.cpp
 * @brief Main application file.
 * @author Shashank Agarwal
 * @author Gagan Space
 * @date 10 April , 2024
 *
 * @license GNU General Public License Version 3
 */

#include <Adafruit_BNO08x.h>
#include <Adafruit_DPS310.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>

#if defined(SERVO)
#define PWMFREQ 50
#define PWMRESOLUTION 8
#define SERVOMAX 255
#define SERVOMIN 0
#define SERVOMID 128
#endif

struct telemetry_t {
  float packetCount;
  uint8_t mode;
  uint8_t state;
  float altitude;
  float temperature;
  float pressure;
  float voltage;
  float gpsTime;
  float gpsLatitude;
  float gpsLongitude;
  uint8_t gpsSats;
  float tiltX;
  float tiltY;
  float rotZ;
} telemetry;

#if defined(VOLTAGE_SENSE)
int voltagePin;
#endif

#if defined(BNO085)
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);

// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

#endif

#if defined(DPS310)
Adafruit_DPS310 baro;
#endif

// put function declarations here:
#if defined(VOLTAGE_SENSE)
void getVoltage(sensors_event_t *voltage_event);
#endif

#if defined(BNO085)
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr,
                       bool degrees = false);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector,
                         euler_t *ypr, bool degrees = false);
void setReports(sh2_SensorId_t reportType, long report_interval);
#endif

#if defined(SERVO)
int servo1Channel;
int servo2Channel;
int servo3Channel;
#endif

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200); // Initialize serial communication

  while (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    yield();
  }

  // Open the JSON file for writing
  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, configFile);
  if (error) {
    Serial.println("Failed to parse config file");
    return;
  }

  // Close the file
  configFile.close();

#if defined(VOLTAGE_SENSE)
  voltagePin = doc["sensors"]["voltage"]["pin"];
#endif

#if defined(DPS310)
  TwoWire *baroWire = NULL;
  int sdaPinBaro = doc["sensors"]["baro"]["sda"];
  int sclPinBaro = doc["sensors"]["baro"]["scl"];
#endif

#if defined(BNO085)
  TwoWire *imuWire = NULL;
  int sdaPinImu = doc["sensors"]["imu"]["sda"];
  int sclPinImu = doc["sensors"]["imu"]["scl"];
#endif

#if defined(VOLTAGE_SENSE)
  pinMode(voltagePin, INPUT);
#endif

#if defined(DPS310) && defined(BNO085)
  if (sdaPinBaro == sdaPinImu && sclPinBaro == sclPinImu) {
    baroWire = &Wire;
    imuWire = &Wire;
  } else if (sdaPinBaro != sdaPinImu && sclPinBaro != sclPinImu) {
    baroWire = &Wire;
    imuWire = &Wire1;
  } else {
    Serial.println("Error: Invalid resource allocation");
    while (1)
      yield();
  }

#elif defined(DPS310)
  baroWire = &Wire;

#elif defined(BNO085)
  imuWire = &Wire;
#endif

#if defined(DPS310)
  baroWire->setPins(sdaPinBaro, sclPinBaro);
  if (!baro.begin_I2C(DPS310_I2CADDR_DEFAULT, baroWire)) {
    Serial.println("Failed to find DPS310");
    while (1)
      yield();
  }
  baro.setMode(DPS310_CONT_PRESTEMP);
  baro.configurePressure(DPS310_1HZ, DPS310_64SAMPLES);
  baro.configureTemperature(DPS310_1HZ, DPS310_64SAMPLES);
  Serial.println("Initialized DPS310");
#endif

#if defined(BNO085)
  imuWire->setPins(sdaPinImu, sclPinImu);
  if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, imuWire)) {
    Serial.println("Failed to find BNO085");
    while (1)
      yield();
  }
  Serial.println("Initialized BNO085");
  setReports(reportType, reportIntervalUs);
#endif

#if defined(SERVO)
  int servo1Pin = doc["actuators"]["Servo1"]["pin"];
  int servo2Pin = doc["actuators"]["Servo2"]["pin"];
  int servo3Pin = doc["actuators"]["Servo3"]["pin"];

  int channelCounter = 0;

  if (servo1Pin != -1) {
    servo1Channel = channelCounter;
    channelCounter++;
    ledcSetup(servo1Channel, PWMFREQ, PWMRESOLUTION);
    ledcAttachPin(servo1Pin, servo1Channel);
    ledcWrite(servo1Channel, SERVOMID);
  }
  if (servo2Pin != -1) {
    servo2Channel = channelCounter;
    channelCounter++;
    ledcSetup(servo2Channel, PWMFREQ, PWMRESOLUTION);
    ledcAttachPin(servo2Pin, servo2Channel);
    ledcWrite(servo2Channel, SERVOMID);
  }
  if (servo3Pin != -1) {
    servo3Channel = channelCounter;
    channelCounter++;
    ledcSetup(servo3Channel, PWMFREQ, PWMRESOLUTION);
    ledcAttachPin(servo3Pin, servo3Channel);
    ledcWrite(servo3Channel, SERVOMID);
  }
#endif
}

void loop() {
// put your main code here, to run repeatedly:
#if defined(VOLTAGE_SENSE)
  sensors_event_t batt_voltage;
#endif

#if defined(DPS310)
  sensors_event_t baro_temperature, baro_pressure;
#endif

#if defined(BNO085)
  sh2_SensorValue_t imuData;
#endif

#if defined(VOLTAGE_SENSE)
  getVoltage(&batt_voltage);
  telemetry.voltage = batt_voltage.voltage;
#endif

#if defined(DPS310)
  if (baro.pressureAvailable() && baro.temperatureAvailable()) {
    baro.getEvents(&baro_temperature, &baro_pressure);
    telemetry.temperature = baro_temperature.temperature;
    telemetry.pressure = baro_pressure.pressure;
    telemetry.altitude = baro.readAltitude();
  }
#endif

#if defined(BNO085)

  if (bno08x.wasReset()) {
    Serial.print("IMU was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&imuData) &&
      imuData.sensorId == SH2_ARVR_STABILIZED_RV) {
    quaternionToEulerRV(&imuData.un.arvrStabilizedRV, &ypr, true);

    telemetry.rotZ = ypr.yaw;
    telemetry.tiltX = ypr.pitch;
    telemetry.tiltY = ypr.roll;
  }
#endif

  Serial.write((uint8_t *)&telemetry, sizeof(telemetry));
  delay(1000);
}

// put function definitions here:

#if defined(VOLTAGE_SENSE)
void getVoltage(sensors_event_t *voltage_event) {
  voltage_event->voltage = map(analogRead(voltagePin), 0, 1024, 0, 5);
}
#endif

#if defined(BNO085)
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr,
                       bool degrees) {

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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector,
                         euler_t *ypr, bool degrees) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i,
                    rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}
#endif