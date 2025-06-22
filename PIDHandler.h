#ifndef PIDHANDLER
#define PIDHANDLER

#include "FlightDataHandler.h"
#include "Config.h"
#include "motor_handler.h"
#include "EEPROMHandler.h"

struct MOTOR_DATA {
  float rFactor, pFactor, yFactor, aFactor;
  int throttle = 48; // 48 - 2047
  void setFact(float r, float p, float y, float a) 
  {
    rFactor = r; pFactor = p; yFactor = y; aFactor = a;
  }
  int calculate(float r, float p, float y, float a)
  {
    float val = r * rFactor + p * pFactor + y * yFactor + a * aFactor;
    if (val < 0) val = 0; else if (val > 100) val = 100;
    throttle = map(val * 100, 0, 10000, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    return throttle;
  }
  void stop(){
    throttle = DSHOT_THROTTLE_MIN;
  }
};
extern MOTOR_DATA motorFL_data;
extern MOTOR_DATA motorFR_data;
extern MOTOR_DATA motorBL_data;
extern MOTOR_DATA motorBR_data;

extern FlightDataHandler flightData; // obiectul de management al senzorilor
extern TaskHandle_t internRegulatorTaskHandle; // task handle
extern TaskHandle_t externRegulatorTaskHandle; // task handle
extern bool isMotorKill;
struct CONTROL_TARGET {
  float pitch, roll, yaw, altitude; // valorile setpoint
  unsigned long lastTime = 0; // timpul actualizarii trecute
   //val [-1; 1] map [-maxPitch; +maxPitch]
  void setPitch(float val) { pitch = MAX_PITCH * val; }
  // val [-1; 1] map [-maxRoll; +maxRoll]
  void setRoll(float val) { roll = MAX_ROLL * val; }
  // creste yaw cu un pas predeterminat si in functie de timp
  void increaseYaw(float val)  //
  {
    yaw += val * YAW_INC_RATE; // creste valoarea setpoint yaw
    if (yaw > 180) yaw = -180; // mai mare de 180 reset
  }
  // creste altitudinea in acelsi fel ca yaw
  void increaseAlt(float val) {
    altitude += val * ALT_INC_RATE; // creste altitudinea 
    altitude = min(MAX_ALT, max(MIN_ALT, altitude)); // clamp al valorilor
  }
  // seteaza setpoint yaw pentru initializare
  void setYaw(float val) { yaw = val; }
  // seteaza setpoint altitudine pentru initializare
  void setAltitude(float val) { altitude = val; }
};
extern CONTROL_TARGET controlTarget;
extern CONTROL_TARGET controlTargetRoC;

struct REGULATOR_DATA {
  String key; // denimire
  float kp, ki, kd; // constantele
  float previousError = 0.0f; // eroarea anterioara folosita pentru calculul derivatei
  float integral = 0.0f; // integrala (suma erorilor * dt)
  double outputMin, outputMax; // valori clamp
  unsigned long lastTime; // pentru calculul dt
  // calculeaza valoareade iesir pe baza setpoint-ului si a masuratorii
  double compute(double setpoint, double measured) {
      unsigned long now = micros();   // ONE micros() call
      double dt = (now - lastTime) / 1e6;
      lastTime = now;  // update AFTER computing dt
      // Protect against dt = 0:
      if (dt <= 0.0f) dt = 1e-6;


      double error = setpoint - measured;
      integral += error * dt;
      double derivative;
      if (error == previousError && dt == 0) derivative = 0;
      else derivative = (error - previousError)  / dt;
      previousError = error;

      double output = kp * error + ki * integral + kd * derivative;

      // Clamp output
      if (output > outputMax) output = outputMax;
      else if (output < outputMin) output = outputMin;
      //Serial.print("Setpoint: "); Serial.print(setpoint); Serial.print(" measured: "); Serial.print(measured); Serial.print(" dt: "); Serial.print(dt, 6); Serial.print(" err: "); Serial.print(error); Serial.print(" deriv: "); Serial.print(derivative); Serial.print(" integral: "); Serial.println(integral);
      return output;
  }

  // resetarea regulatorului 
  void reset() { previousError = 0.0f; integral = 0.0f; }
};
// Regulatoarele care compun regulatorul buclei interne
extern REGULATOR_DATA pitchRegulatorIntern;
extern REGULATOR_DATA rollRegulatorIntern;
extern REGULATOR_DATA yawRegulatorIntern;
extern REGULATOR_DATA altitudeRegulatorIntern;
// Regulatoarele care compun regulatorul buclei externe
extern REGULATOR_DATA pitchRegulatorExtern;
extern REGULATOR_DATA rollRegulatorExtern;
extern REGULATOR_DATA yawRegulatorExtern;
extern REGULATOR_DATA altitudeRegulatorExtern;
// returneaza constantele sub forma de string, folosit pentru transmiterea UDP
String getRegulatorConstants(REGULATOR_DATA& reg); 
// seteaza constantele regulatorului 'reg'
void setRegulatorConstants(REGULATOR_DATA& reg, float kp, float ki, float kd);
void printRegulatorsConstants(); // afiseaza in consola valorile regulatoarelor
void PIDBegin(); // incarca valorile din EEPROM in regulatoare si porneste task-ul paralel
void resetRegulators(); // reseteaza regulatoarele
void setInternControlTarget(float pitch, float roll, float yaw, float altitude); // seteaza setPoint-ul intern
void setExternControlTarget(float pitch, float roll, float yaw, float altitude); // seteaza setPoint-ul extern
void updateInternRegulator(); // iteratia regulatoarelor
void updateExternRegulator(); // iteratia regulatoarelor
void internRegulatorTask(void* pvParameters); // task-ul propriu-zis
void externRegulatorTask(void* pvParameters); // task-ul propriu-zis
void loadRegulatorData(REGULATOR_DATA& reg, String kp_key, String ki_key, String kd_key);
float getRegulatorKP(REGULATOR_DATA& reg); // returneaza valoarea kp a regulatorlui precizat
float getRegulatorKI(REGULATOR_DATA& reg); // returneaza valoarea ki a regulatorlui precizat
float getRegulatorKD(REGULATOR_DATA& reg); // returneaza valoarea kd a regulatorlui precizat
#endif