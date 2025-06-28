#include "FlightDataHandler.h"

// constructorul gol
FlightDataHandler::FlightDataHandler() {
  // Initialize mutex in constructor
  dataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
}

// initializarea clasei
void FlightDataHandler::begin() {
  Wire.begin();  // initilizarea comunicatiei I2C pe pinii impliciti
  
  // Initialize sensor data to safe values
  sensorData.roll = 0.0;
  sensorData.pitch = 0.0;
  sensorData.yaw = 0.0;
  sensorData.altitude = 0.0;
  
  lastSensorData = sensorData;
  sensorRoC.roll = 0.0;
  sensorRoC.pitch = 0.0;
  sensorRoC.yaw = 0.0;
  sensorRoC.altitude = 0.0;
  
  beginIMU(); // initializarea icm20948
  beginL0X(); // initializarea vl53l0x

  // creeaza task-ul de citire
  xTaskCreatePinnedToCore(
    sensorTask,         // functia
    "SensorTask",       // numele
    4096,               // dimensiunea
    this,               // parametrii
    2,                  // prioritatea (increased for sensor task)
    &sensorTaskHandle,  // handle-ul
    1                   // core 1
  );
}

void FlightDataHandler::sensorTask(void* pvParameters) {
  while (true) {
    FlightDataHandler* handler = static_cast<FlightDataHandler*>(pvParameters);
    
    // Protect I2C access with mutex
    if (xSemaphoreTake(handler->i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      // get start time in microseconds
      int64_t startTime = esp_timer_get_time();
      // perform read
      handler->readData();
      // release I2C mutex
      xSemaphoreGive(handler->i2cMutex);
      
      // delay - reduced to improve responsiveness
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      // If can't get mutex, just delay and try again
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// initializarea senzorului vl53l0x
void FlightDataHandler::beginL0X() {
  // seteaza valoare de timeout
  L0X.setTimeout(500);
  // initializeaza senzorl, daca esueaza reseteaza esp-ul
  if (!L0X.init()) { esp_restart(); } 
  // porneste senzorul in mod continuu
  L0X.startContinuous(100);
}

// initializarea senzorului icm20948
void FlightDataHandler::beginIMU() {
  bool initialized = false;
  // bucla infinita pana la initializarea senzorului
  while (!initialized) 
  {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) { delay(500); }
     else { initialized = true; }
  }
  success = true;
  // Inițializează filtrul DMP
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Activează senzorul DMP dorit (Game Rotation Vector)
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // Setează rata de actualizare a datelor (Output Data Rate) pentru senzorul activat
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  // Activează FIFO și modul DMP
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Resetează bufferul FIFO după activarea completă a funcțiilor
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  // in caz de esec reseteaza esp-ul
  if (!success) { esp_restart(); }
}

// citeste si calculeaza inclinatia, converteste din quaternion in grade euler
void FlightDataHandler::readIMUData() {
  // tip de data (struct) definit in librarie
  icm_20948_DMP_data_t data;
  // citest datele din buffer-ul fifo
  myICM.readDMPdataFromFIFO(&data);
  // sunt date valide inca accesibile
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    // verificare daca pachetulcontine quaternioni
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      // conversia in 'double', imparte la 2^30
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; 
      
      // Validate quaternion components
      if (isnan(q1) || isnan(q2) || isnan(q3) || 
          isinf(q1) || isinf(q2) || isinf(q3)) {
        return; // Skip this reading if invalid
      }
      
      // scalar al quaternionului, asigura lungimea 1 a quat. complet
      double q_sum_squares = (q1 * q1) + (q2 * q2) + (q3 * q3);
      if (q_sum_squares >= 1.0) {
        return; // Invalid quaternion, skip
      }
      
      double q0 = sqrt(1.0 - q_sum_squares);
      
      // salvarea in variabile temporare, conversia in ordinea rotatiilor 3-2-1
      double qw = q0;
      double qx = q2;
      double qy = q1;
      double qz = -q3; // inverseaza sensul axei Z
      
      // roll (axa X)
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = atan2(t0, t1) * 180.0 / PI; // conversie in grade
      
      // pitch (axa Y)
      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI; // conversie in grade
      
      // yaw (axa Z)
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI; // conversie in grade
      
      // Final validation before storing
      if (isnan(roll) || isnan(pitch) || isnan(yaw) ||
          isinf(roll) || isinf(pitch) || isinf(yaw)) {
        return; // Skip if any calculated value is invalid
      }
      
      // Protect data access with mutex
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // salvarea datelor vechi
        lastSensorData.roll = sensorData.roll;
        lastSensorData.pitch = sensorData.pitch;
        lastSensorData.yaw = sensorData.yaw;
        
        // salvarea datelor in structura
        sensorData.roll = roll;
        sensorData.pitch = pitch;
        sensorData.yaw = yaw;
        
        unsigned long now = micros();
        double dt = (double)(now - lastRoCTime) / 1e6; // us -> s
        lastRoCTime = now;
        
        // Calculate rate of change only if dt is reasonable
        if (dt > 0.0001 && dt < 1.0) { // Between 0.1ms and 1s
          sensorRoC.roll = roll_pt1.calculate((sensorData.roll - lastSensorData.roll) / dt);
          sensorRoC.pitch = pitch_pt1.calculate((sensorData.pitch - lastSensorData.pitch) / dt);
          sensorRoC.yaw = (sensorData.yaw - lastSensorData.yaw) / dt;
          
          // Limit rate of change to reasonable values
          sensorRoC.roll = constrain(sensorRoC.roll, -1000, 1000);
          sensorRoC.pitch = constrain(sensorRoC.pitch, -1000, 1000);
          sensorRoC.yaw = constrain(sensorRoC.yaw, -1000, 1000);
        }
        
        xSemaphoreGive(dataMutex);
      }
    }
  }
}

// citeste altitudinea si o salveaza
void FlightDataHandler::readL0XData() {
  float newAltitude = L0X.readRangeContinuousMillimeters();
  
  // Validate reading (VL53L0X returns 65535 for out of range/error)
  if (newAltitude >= 65535 || newAltitude < 0) {
    return; // Skip invalid reading
  }
  
  // Protect data access with mutex
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    lastSensorData.altitude = sensorData.altitude;
    sensorData.altitude = newAltitude;
    
    unsigned long now = micros();
    float dt = (float)(now - lastL0XRead) / 1e6; // us -> s
    lastL0XRead = now / 1e3;;
    
    // Calculate rate of change only if dt is reasonable
    if (dt > 0.01 && dt < 10.0) { // Between 10ms and 10s
      sensorRoC.altitude = (sensorData.altitude - lastSensorData.altitude) / dt;
      // Limit rate of change to reasonable values
      sensorRoC.altitude = constrain(sensorRoC.altitude, -10000, 10000);
    }
    
    xSemaphoreGive(dataMutex);
  }
}

// apeleaza citirea ambilor senzori
void FlightDataHandler::readData() {
  unsigned long now = millis(); // timpul actual
  // citirea distantei are o frecventaredusa fata de citirea orientarii
  if ((float)(now) > L0X_READ_INTERVAL_MS + lastL0XRead) { 
    readL0XData();
  } else { 
    readIMUData(); 
  }
}

// Thread-safe getters with data validation
float FlightDataHandler::getRoll() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorData.roll;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}

float FlightDataHandler::getPitch() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorData.pitch;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}

float FlightDataHandler::getYaw() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorData.yaw;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}

float FlightDataHandler::getAltitude() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorData.altitude;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}

float FlightDataHandler::getRollRate() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorRoC.roll;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}

float FlightDataHandler::getPitchRate() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorRoC.pitch;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}
float FlightDataHandler::getYawRate() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorRoC.yaw;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}
float FlightDataHandler::getAltitudeRate() { 
  float value = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    value = sensorRoC.altitude;
    xSemaphoreGive(dataMutex);
  }
  return isnan(value) || isinf(value) ? 0.0 : value;
}
void FlightDataHandler::setA(float val){
  roll_pt1.setA(val);
  pitch_pt1.setA(val);
}
// ... similar for pitch, yaw, altitude rates
// ... similar for pitch, yaw, altitude rates
// ... similar for pitch, yaw, altitude rates
// ... similar for pitch, yaw, altitude rates