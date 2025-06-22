#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include "ICM_20948.h"
#include <Wire.h>
#include <vector>
#include <cmath>
#include <VL53L0X.h>
#include "Config.h"

#define AD0_VAL 1 // valoarea pinului AD0
#define L0X_READ_INTERVAL_MS 100 // interval de citire al distantei
// strutura in care sunt memorate datele senzorilor
struct SensorData {
  double yaw, pitch, roll; // orientarea in cele 3 axe
  double altitude; // altitudinea, distanta pana la sol 
};
// clasa responsabila pentru management
class FlightDataHandler {
public:
  SensorData sensorData;
  SensorData lastSensorData;
  SensorData sensorRoC;
  FlightDataHandler(); // constructor
  void begin(); // initializeaza senzorii
  void readData(); // citeste datele si calculeaza orientarea
  float getPitch(); // returneaza inclinatia
  float getRoll(); // returneaza rotatia
  float getYaw(); // returneaza directia
  float getAltitude(); // returneaza altitudinea
  bool success; // indica initializareacu success sau nu a senzorilor
  TaskHandle_t sensorTaskHandle; // handle al task-ului de citire
  // Add these to your FlightDataHandler class:
  float getRollRate();
  float getPitchRate(); 
  float getYawRate();
  float getAltitudeRate();
// Add these member variables to your class:
private:
  SemaphoreHandle_t dataMutex;  // For protecting sensor data
  SemaphoreHandle_t i2cMutex;  // For protecting I2C bus access
  ICM_20948_I2C myICM; // obiect folosit pentru senzorul icm, apartine librariei icm20948
  VL53L0X L0X; // obiect folosit pentru senzorul de sitanta, apartine librariei vl53l0x
  void readIMUData(); // citeste datele icm
  void readL0XData(); // citeste datele l0x
  void beginIMU(); // initializeaza icm
  void beginL0X(); // initializeaza l0x
  float lastAltitReadTime; // ms a ultimei citiri a altitudinii
  unsigned long lastRoCTime; // us a ultimei citiri a imu
  float dt_ms; // diferenta de timp intre iteratia trecuta si cea actuala (ms)
  unsigned long lastL0XRead; // la fel ca 'lastTime' dar pentru senzorul de distanta
  static void sensorTask(void* pvParameters); // task-ul de citire
};
#endif
