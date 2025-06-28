#include "PIDHandler.h"

bool regStart;

MOTOR_DATA motorFL_data;
MOTOR_DATA motorFR_data;
MOTOR_DATA motorBL_data;
MOTOR_DATA motorBR_data;

TaskHandle_t internRegulatorTaskHandle; // handle
TaskHandle_t externRegulatorTaskHandle; // task handle
FlightDataHandler flightData; // clasa senzorilor
CONTROL_TARGET controlTarget; // setpoint-ul pozitiei
CONTROL_TARGET controlTargetRoC; // setpoint-ul derivatei
bool isMotorKill;
// regulatoarele buclei interne
REGULATOR_DATA pitchRegulatorIntern, rollRegulatorIntern;
REGULATOR_DATA yawRegulatorIntern, altitudeRegulatorIntern;
// regulatoarele buclei externe
REGULATOR_DATA pitchRegulatorExtern, rollRegulatorExtern;
REGULATOR_DATA yawRegulatorExtern, altitudeRegulatorExtern;

// initializeaza constantele regulatoarelor si creeaza task-ul
void PIDBegin() {
  // Initialize flight data handler first
  flightData.begin();
  
  // regulatorul intern
  // seteaza cheia fiecarui regulator si valorile minime si maxime
  rollRegulatorIntern.key     = "ROLL"; // ROLL
  rollRegulatorIntern.outputMin = -30.0; rollRegulatorIntern.outputMax = 30.0;
  pitchRegulatorIntern.key    = "PITCH"; // PITCH
  pitchRegulatorIntern.outputMin = -30.0; pitchRegulatorIntern.outputMax = 30.0;
  yawRegulatorIntern.key      = "YAW"; // YAW
  yawRegulatorIntern.outputMin = -10.0; yawRegulatorIntern.outputMax = 10.0;
  altitudeRegulatorIntern.key = "ALTITUDE"; // ALTITUDE
  altitudeRegulatorIntern.outputMin = -50.0; altitudeRegulatorIntern.outputMax = 50.0;
  // regulatorul extern
  // seteaza cheia fiecarui regulator si valorile minime si maxime
  rollRegulatorExtern.key     = "ROLL_E"; // ROLL
  rollRegulatorExtern.outputMin = -10.0; rollRegulatorExtern.outputMax = 10.0; // Fixed: was rollRegulatorIntern
  pitchRegulatorExtern.key    = "PITCH_E"; // PITCH
  pitchRegulatorExtern.outputMin = -10.0; pitchRegulatorExtern.outputMax = 10.0; // Fixed: was pitchRegulatorIntern
  yawRegulatorExtern.key      = "YAW_E"; // YAW
  yawRegulatorExtern.outputMin = -10.0; yawRegulatorExtern.outputMax = 10.0; // Fixed: was yawRegulatorIntern
  altitudeRegulatorExtern.key = "ALTITUDE_E"; // ALTITUDE
  altitudeRegulatorExtern.outputMin = -20.0; altitudeRegulatorExtern.outputMax = 20.0; // Fixed: was altitudeRegulatorIntern
  
  // regulatorul intern
  // incarca valorile constante ale regulatorului roll
  loadRegulatorData(rollRegulatorIntern, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD);
  // incarca valorile constante ale regulatorului pitch
  loadRegulatorData(pitchRegulatorIntern, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
  // incarca valorile constante ale regulatorului yaw
  loadRegulatorData(yawRegulatorIntern, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD);
  // incarca valorile constante ale regulatorului altitude
  loadRegulatorData(altitudeRegulatorIntern, ALT_PID_KP, ALT_PID_KI, ALT_PID_KD);
  // regulatorul extern
  // incarca valorile constante ale regulatorului roll
  loadRegulatorData(rollRegulatorExtern, ROLL_PID_KP_E, ROLL_PID_KI_E, ROLL_PID_KD_E);
  // incarca valorile constante ale regulatorului pitch
  loadRegulatorData(pitchRegulatorExtern, PITCH_PID_KP_E, PITCH_PID_KI_E, PITCH_PID_KD_E);
  // incarca valorile constante ale regulatorului yaw
  loadRegulatorData(yawRegulatorExtern, YAW_PID_KP_E, YAW_PID_KI_E, YAW_PID_KD_E);
  // incarca valorile constante ale regulatorului altitude
  loadRegulatorData(altitudeRegulatorExtern, ALT_PID_KP_E, ALT_PID_KI_E, ALT_PID_KD_E);
  printRegulatorsConstants();

  flightData.setA((double)loadFloat(A_PT1_KEY, 0.0));
  
  // Wait a bit for sensors to stabilize
  delay(1000);

  motorFL_data.setFact(-1, -1, +1, +1);
  motorFR_data.setFact(+1, -1, -1, +1);
  motorBL_data.setFact(-1, +1, -1, +1);
  motorBR_data.setFact(+1, +1, +1, +1);
  
  // valorile initiale ale setpoint-ului yaw si altitudine
  controlTarget.setYaw(flightData.getYaw()); 
  controlTarget.setAltitude(flightData.getAltitude());
  
  // creeaza task-ul loop pid
  xTaskCreatePinnedToCore(
    internRegulatorTask,        // functia
    "internRegulatorTask",      // denumirea
    4096,                 // dimensiunea
    NULL,                 // parametrul
    3,                    // prioritatea (reduced from 5 to avoid blocking sensor task)
    &internRegulatorTaskHandle, // handle
    0                     // core 0 (moved to different core than sensors)
  );
}

// actualizeaza PID-urile de viteza unghiulara
void updateRegulator() {
  // drona este la sol si nu se ridica
  // currentAltitude < 200 && controlTarget.altitude < 200 || 
  if (!regStart) 
  {
    set_throttle(0, 48);
    set_throttle(1, 48);
    set_throttle(2, 48);
    set_throttle(3, 48);
    resetRegulators(); // reseteaza regulatoarele
    return;
  }
  
  // regulator extern
  // obtine valorile actuale ale pozitiei - USING THREAD-SAFE GETTERS
  double currentRoll     = flightData.getRoll();
  double currentPitch    = flightData.getPitch();
  double currentYaw      = flightData.getYaw();
  double currentAltitude = flightData.getAltitude();
  
  // Validate sensor readings before using them
  if (isnan(currentRoll) || isnan(currentPitch) || isnan(currentYaw) || isnan(currentAltitude) ||
      isinf(currentRoll) || isinf(currentPitch) || isinf(currentYaw) || isinf(currentAltitude)) {
    Serial.println("Warning: Invalid sensor data, skipping PID update");
    return;
  }
  double outRoll, outPitch, outYaw, outAltitude;
  
  // calculeaza output-ul regulatorului extern
  outRoll     = rollRegulatorExtern.kp * (controlTarget.roll - currentRoll);
  outPitch    = pitchRegulatorExtern.kp * (controlTarget.pitch - currentPitch);
  outYaw      = yawRegulatorExtern.kp * (controlTarget.yaw - currentYaw);
  outAltitude = altitudeRegulatorExtern.kp * (controlTarget.altitude - currentAltitude);
  
  // Validate extern regulator outputs
  if (isnan(outRoll) || isnan(outPitch) || isnan(outYaw) || isnan(outAltitude)) {
    Serial.println("Warning: Invalid extern regulator output");
    return;
  }
  
  // seteaza setpoint-ul regulatorului intern
  controlTargetRoC.setRoll(outRoll); 
  controlTargetRoC.setPitch(outPitch); 
  controlTargetRoC.setYaw(outYaw); 
  controlTargetRoC.setAltitude(outAltitude);  
  
  // obtine valorile reale derivate
  float currentRollRate = flightData.getRollRate();
  float currentPitchRate = flightData.getPitchRate();
  float currentYawRate = flightData.getYawRate();
  float currentAltitudeRate = flightData.getAltitudeRate();
  
  // Validate rate readings
  if (isnan(currentRollRate) || isnan(currentPitchRate) || isnan(currentYawRate) || isnan(currentAltitudeRate) ||
      isinf(currentRollRate) || isinf(currentPitchRate) || isinf(currentYawRate) || isinf(currentAltitudeRate)) {
    Serial.println("Warning: Invalid rate sensor data, using zero rates");
    currentRollRate = 0.0;
    currentPitchRate = 0.0;
    currentYawRate = 0.0;
    currentAltitudeRate = 0.0;
  }
  
  // calculeaza iesirea fiecarui regulator intern
  outRoll     = rollRegulatorIntern.compute(controlTargetRoC.roll, currentRollRate);
  outPitch    = pitchRegulatorIntern.compute(controlTargetRoC.pitch, currentPitchRate);
  outYaw      = yawRegulatorIntern.compute(controlTargetRoC.yaw, currentYawRate);
  outAltitude = altitudeRegulatorIntern.compute(controlTargetRoC.altitude, currentAltitudeRate);
  
  // Final validation of PID outputs
  if (isnan(outRoll) || isnan(outPitch) || isnan(outYaw) || isnan(outAltitude) ||
      isinf(outRoll) || isinf(outPitch) || isinf(outYaw) || isinf(outAltitude)) {
    Serial.println("Warning: Invalid PID output, resetting regulators");
    resetRegulators();
    return;
  }
  
  // calculeaza viteza fiecarui motor
  int thFL = motorFL_data.calculate(outRoll, outPitch, outYaw, outAltitude);
  int thFR = motorFR_data.calculate(outRoll, outPitch, outYaw, outAltitude);
  int thBL = motorBL_data.calculate(outRoll, outPitch, outYaw, outAltitude);
  int thBR = motorBR_data.calculate(outRoll, outPitch, outYaw, outAltitude);
  
  // Validate motor outputs
  if (thFL < 0 || thFR < 0 || thBL < 0 || thBR < 0 ||
      thFL > 2000 || thFR > 2000 || thBL > 2000 || thBR > 2000) {
    Serial.println("Warning: Invalid motor output values");
    resetRegulators();
    return;
  }
  
  //seteaza viteza motoarelor
  set_throttle(0, thFL);
  set_throttle(1, thBL);
  set_throttle(2, thBR);
  set_throttle(3, thFR);

  // Serial.print(outRoll); Serial.print(", ");
  // Serial.print(outPitch); Serial.print(", ");
  // Serial.print(outYaw); Serial.print(", ");
  // Serial.print(outAltitude); Serial.print(", ");
  // Serial.print(thFL); Serial.print(", ");
  // Serial.print(thFR); Serial.print(", ");
  // Serial.print(thBL); Serial.print(", ");
  // Serial.println(thBR);
}

// bucla interna, asigura rulaj al codului la frecventa specificata
void internRegulatorTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / INTERN_REG_FREQ);
  while (true) {
    updateRegulator();
    // Use proper task delay timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setExternControlTarget(float pitch, float roll, float yaw, float altitude) {
  // Add validation for control targets
  if (isnan(pitch) || isnan(roll) || isnan(yaw) || isnan(altitude) ||
      isinf(pitch) || isinf(roll) || isinf(yaw) || isinf(altitude)) {
    Serial.println("Warning: Invalid control target values");
    return;
  }
  
  controlTarget.pitch = pitch;
  controlTarget.roll = roll;
  controlTarget.yaw = yaw;
  controlTarget.altitude = altitude;
}

void setInternControlTarget(float pitch, float roll, float yaw, float altitude) {
  // Add validation for control targets
  if (isnan(pitch) || isnan(roll) || isnan(yaw) || isnan(altitude) ||
      isinf(pitch) || isinf(roll) || isinf(yaw) || isinf(altitude)) {
    Serial.println("Warning: Invalid internal control target values");
    return;
  }
  
  controlTargetRoC.pitch = pitch;
  controlTargetRoC.roll = roll;
  controlTargetRoC.yaw = yaw;
  controlTargetRoC.altitude = altitude;
}

// salveaza valorile in eeprom folosind functiile definite in eepromhandler
void saveConstantsInEEPROM(String key_kp, String key_ki, String key_kd, 
                            float val_kp, float val_ki, float val_kd) {
  saveFloat(key_kp, val_kp);
  saveFloat(key_ki, val_ki);
  saveFloat(key_kd, val_kd);
}

// seteaza noi valoriin regulatorul primit si le salveaza in eeprom
void setRegulatorConstants(REGULATOR_DATA& reg, float kp, float ki, float kd) {
  // Validate regulator constants
  if (isnan(kp) || isnan(ki) || isnan(kd) || isinf(kp) || isinf(ki) || isinf(kd)) {
    Serial.println("Warning: Invalid regulator constants");
    return;
  }
  
  reg.kp = kp; // noile valori
  reg.ki = ki;
  reg.kd = kd;
  if (reg.key == "ROLL") { // salvare in eeprom in functie de regulator
    saveConstantsInEEPROM(ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "PITCH") {
    saveConstantsInEEPROM(PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "YAW") {
    saveConstantsInEEPROM(YAW_PID_KP, YAW_PID_KI, YAW_PID_KD, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "ALTITUDE") {
    saveConstantsInEEPROM(ALT_PID_KP, ALT_PID_KI, ALT_PID_KD, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "ROLL_E") { // salvare in eeprom in functie de regulator
    saveConstantsInEEPROM(ROLL_PID_KP_E, ROLL_PID_KI_E, ROLL_PID_KD_E, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "PITCH_E") {
    saveConstantsInEEPROM(PITCH_PID_KP_E, PITCH_PID_KI_E, PITCH_PID_KD_E, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "YAW_E") {
    saveConstantsInEEPROM(YAW_PID_KP_E, YAW_PID_KI_E, YAW_PID_KD_E, 
                          reg.kp, reg.ki, reg.kd);
  } else if (reg.key == "ALTITUDE_E") {
    saveConstantsInEEPROM(ALT_PID_KP_E, ALT_PID_KI_E, ALT_PID_KD_E, 
                          reg.kp, reg.ki, reg.kd);
  }
}

void printRegulatorsConstants() {
  Serial.println("Pitch Reg: " + getRegulatorConstants(pitchRegulatorIntern));
  Serial.println("Roll Reg: " + getRegulatorConstants(rollRegulatorIntern));
  Serial.println("Yaw Reg: " + getRegulatorConstants(yawRegulatorIntern));
  Serial.println("Altit Reg: " + getRegulatorConstants(altitudeRegulatorIntern));
  Serial.println("----------------");
  Serial.println("Pitch Reg: " + getRegulatorConstants(pitchRegulatorExtern));
  Serial.println("Roll Reg: " + getRegulatorConstants(rollRegulatorExtern));
  Serial.println("Yaw Reg: " + getRegulatorConstants(yawRegulatorExtern));
  Serial.println("Altit Reg: " + getRegulatorConstants(altitudeRegulatorExtern));
}

String getRegulatorConstants(REGULATOR_DATA& reg) 
{
  return ("kp: " + String(reg.kp) + " ki: " + String(reg.ki) + " kd: " + String(reg.kd)); 
}

// returneaza valorile regulatorului primit
float getRegulatorKP(REGULATOR_DATA& reg) { return reg.kp; }
float getRegulatorKI(REGULATOR_DATA& reg) { return reg.ki; }
float getRegulatorKD(REGULATOR_DATA& reg) { return reg.kd; }

// reseteaza toate regulatoarele
void resetRegulators() {
  pitchRegulatorIntern.reset();
  rollRegulatorIntern.reset();
  yawRegulatorIntern.reset();
  altitudeRegulatorIntern.reset();

  motorFL_data.stop();
  motorFR_data.stop();
  motorBL_data.stop();
  motorBR_data.stop();
}

void loadRegulatorData(REGULATOR_DATA& reg, String kp_key, String ki_key, String kd_key) {
  float kp = loadFloat(kp_key, 0.0); // functia definita in eepromhandler
  float ki = loadFloat(ki_key, 0.0);
  float kd = loadFloat(kd_key, 0.0);
  setRegulatorConstants(reg, kp, ki, kd);
}