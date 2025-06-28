#include "PIDHandler.h"
#include "DualSEnseHandler.h"
#include "UdpHandler.h"
#include "BatteryLevelReader.h"

#define NETWORK_CONNECTION true // false pentru a opri comunicatia Wi-Fi

void sendRegDataAsJson(String reg);  // trimite json cu valorile regulatorului ales
String createJson(); // creeazaun json cu toate datele necesare transmiterii udp
float applyDeadzone(float val); // aplica zona moarta asupra valorii primite
void sendRecvFlightDataUDP(); // gestioneaza fluxul de date udp
void processControllerInput(); // seteaza setpoint-ul pe baza input-ului de la joystick
void sendRegDataAsJson(String reg); // trimite valorile pid ale regulatorului ales
void UDPTask(void* parameters);

void setup() {
  // Setează CPU la 260 MHz
  Serial.begin(115200);
  setCpuFrequencyMhz(260);
  motor_init();
  delay(1000);

  //initDualSenseController(); // incepe conexiunea Bluetooth
  analogReadResolution(12); // rezolutie pe 12 biti pentru citirea ADC
  batteryReaderInit(); // porneste procesul de citit al bateriei
  if (NETWORK_CONNECTION) setupUDP(); // porneste conexiunea Wi-Fi
  delay(100); // intarziere de 500ms

  // se initializeazatoate motoarle
  // se initializeaza senzorii si task-ul aferent
  // flightData.begin();
  PIDBegin();

  if (NETWORK_CONNECTION) 
  {
    // trimite valorile regulatoarelor
    while (WiFi.status() != WL_CONNECTED); // asteapta conexiunea Wi-Fi
    delay(100); sendRegDataAsJson("PITCH");
    delay(100); sendRegDataAsJson("ALTITUDE");
    delay(100); sendRegDataAsJson("ROLL");
    delay(100); sendRegDataAsJson("YAW");
    delay(100); sendRegDataAsJson("PITCH_E");
    delay(100); sendRegDataAsJson("ALTITUDE_E");
    delay(100); sendRegDataAsJson("ROLL_E");
    delay(100); sendRegDataAsJson("YAW_E");
  }
  xTaskCreatePinnedToCore(
    UDPTask,            // Task function
    "Task UDP",         // Name of task
    4096,               // Stack size
    NULL,               // Parameter
    1,                  // Priority
    NULL,               // Task handle
    0                   // Core 0
  );
}
void loop() {}

void UDPTask(void* parameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 20 ms = 50 Hz
    while (true) {
        if (NETWORK_CONNECTION) sendRecvFlightDataUDP();
        //processControllerInput();
        vTaskDelay(20);
    }
}

String createJson() {
  String json = "{";
  // // datele pozitionale
  json += "\"roll\":" + String(flightData.getRoll(), 16) + ",";
  json += "\"pitch\":" + String(flightData.getPitch(), 16) + ",";
  json += "\"yaw\":" + String(flightData.getYaw(), 16) + ",";
  json += "\"altitude\":" + String(flightData.getAltitude(), 16) + ",";
  // json += "\"roll\":" +     String(map(motorFR_data.throttle, 48, 2047, 0, 100)) + ",";
  // json += "\"pitch\":" +    String(map(motorBR_data.throttle, 48, 2047, 0, 100)) + ",";
  // json += "\"yaw\":" +      String(map(motorBL_data.throttle, 48, 2047, 0, 100)) + ",";
  // json += "\"altitude\":" + String(map(motorFL_data.throttle, 48, 2047, 0, 100)) + ",";
  // datele RoC
  json += "\"roll_dt\":" +     String(flightData.getRollRate(), 16) + ",";
  json += "\"pitch_dt\":" +    String(flightData.getPitchRate(), 16) + ",";
  json += "\"yaw_dt\":" +      String(flightData.getYawRate(), 16) + ",";
  json += "\"altitude_dt\":" + String(flightData.getAltitudeRate(), 16) + ",";
  // setpoint-uri pozitionale
  json += "\"sp_roll\":" +     String(controlTarget.roll, 16) + ",";
  json += "\"sp_pitch\":" +    String(controlTarget.pitch, 16) + ",";
  json += "\"sp_yaw\":" +      String(controlTarget.yaw, 16) + ",";
  json += "\"sp_altitude\":" + String(controlTarget.altitude, 16) + ",";
  // setpoint RoC
  json += "\"sp_dt_roll\":" +     String(controlTargetRoC.roll, 16) + ",";
  json += "\"sp_dt_pitch\":" +    String(controlTargetRoC.pitch, 16) + ",";
  json += "\"sp_dt_yaw\":" +      String(controlTargetRoC.yaw, 16) + ",";
  json += "\"sp_dt_altitude\":" + String(controlTargetRoC.altitude, 16);
  json += "}";
  return json;
}
// aplica zona moarta a joystick-ului
float applyDeadzone(float val) {
  // definit in config
  if (abs(val) < STICK_DEADZONE) { return 0; } 
  else { return val; }
}
// gestioneaza fluxul de date udp
void sendRecvFlightDataUDP() {
  if (readUDP()) {
    //vTaskSuspend(flightData.sensorTaskHandle);
    if (jsonDoc.containsKey("kbd")){
      Serial.println("kbd");
      int w = jsonDoc["w"];
      int a = jsonDoc["a"];
      int s = jsonDoc["s"];
      int d = jsonDoc["d"];
      int space = jsonDoc["space"];
      int shift = jsonDoc["shift"];

      // isMotorKill = space; 
      if (shift) regStart = !regStart;

      // recupereaza datele joystick-ului
      float throttle = space * 300;
      float brake    = shift * 300;
      //float StickX   = w * 0.7;
      //float StickY   = ControllerInputData.StickY;
      //float StickRX  = ControllerInputData.StickRX;
      // seteaza setpoint-ul pozitiei
      // seteaza setpoint-ul vitezei unghiulare
      setExternControlTarget((w - s) * 10, (a - d) * 10, 0, 0);
      //setInternControlTarget((w - s) * 30, (a - d) * 30, 0, 0);
      controlTarget.increaseAlt(throttle);
      controlTarget.increaseAlt(brake);
      return;
    }
    if (jsonDoc.containsKey("a"))
    {
      double a = jsonDoc["a"].as<double>();
      Serial.println(a);

      flightData.setA(a);
      saveFloat(A_PT1_KEY, (float)a);
      return;
    }
    // daca primeste un json care contine valori pid
    String reg = jsonDoc["reg"].as<String>();
    // salveaza local valorile
    float kp = jsonDoc["kp"].as<float>();
    float ki = jsonDoc["ki"].as<float>();
    float kd = jsonDoc["kd"].as<float>();
    Serial.println(reg);
    // alege regulatorul complet in functie de continutul primit
    // si se fac modificarile in regulatoare
    if (reg == "ROLL") {
      setRegulatorConstants(rollRegulatorIntern, kp, ki, kd);
    } else if (reg == "PITCH") {
      setRegulatorConstants(pitchRegulatorIntern, kp, ki, kd);
    } else if (reg == "YAW") {
      setRegulatorConstants(yawRegulatorIntern, kp, ki, kd);
    } else if (reg == "ALTITUDE") {
      setRegulatorConstants(altitudeRegulatorIntern, kp, ki, kd);
    } else if (reg == "ROLL_E") {
      setRegulatorConstants(rollRegulatorExtern, kp, ki, kd);
    } else if (reg == "PITCH_E") {
      setRegulatorConstants(pitchRegulatorExtern, kp, ki, kd);
    } else if (reg == "YAW_E") {
      setRegulatorConstants(yawRegulatorExtern, kp, ki, kd);
    } else if (reg == "ALTITUDE_E") {
      setRegulatorConstants(altitudeRegulatorExtern, kp, ki, kd);
    } else {
      return;
    }
    sendRegDataAsJson(reg); // trimite inapoi schimbarile efectuate
    //vTaskResume(flightData.sensorTaskHandle);
  } else {
    // altfel trimite valorile masurate de senzori
    sendUDP(createJson());
    //Serial.println(createJson());
    //sendUDP(PID_Motors_Json());
  }
}
// seteaza setpoint-ul pe baza input-ului de la joystick
void processControllerInput() {
  // kill switch, trebuie tinut apasat butonul A
  isMotorKill = ControllerInputData.kill; 

  // recupereaza datele joystick-ului
  float throttle = ControllerInputData.Throttle;
  float brake    = ControllerInputData.Brake;
  float StickX   = ControllerInputData.StickX;
  float StickY   = ControllerInputData.StickY;
  float StickRX  = ControllerInputData.StickRX;
  // D-pad
  bool dpadUp    = ControllerInputData.dpadUp;
  bool dpadDown  = ControllerInputData.dpadDown;
  bool dpadLeft  = ControllerInputData.dpadLeft;
  bool dpadRight = ControllerInputData.dpadRight;
  // seteaza setpoint-ul pozitiei
  controlTarget.setPitch(-applyDeadzone(StickY));
  controlTarget.setRoll(applyDeadzone(StickX));
  controlTarget.increaseYaw(applyDeadzone(StickRX));
  controlTarget.increaseAlt(applyDeadzone(throttle - brake));
  // seteaza setpoint-ul vitezei unghiulare
  controlTargetRoC.setPitch((dpadUp) ? 3 : ((dpadDown) ? -3 : 0));
  controlTargetRoC.setRoll((dpadRight) ? -3 : ((dpadLeft) ? 3 : 0));
}
// trimite valorile pid ale regulatorului ales
void sendRegDataAsJson(String reg) {
  float kp, ki, kd; // constantele
  REGULATOR_DATA regulator; // variabila locala regulator
  // selcteaza regulatorul corect
  if (reg == "ROLL"){ regulator = rollRegulatorIntern; } 
  else if (reg == "PITCH") { regulator = pitchRegulatorIntern; } 
  else if (reg == "YAW") { regulator = yawRegulatorIntern; }
  else if (reg == "ALTITUDE") { regulator = altitudeRegulatorIntern; }
  else if (reg == "ROLL_E"){ regulator = rollRegulatorExtern; } 
  else if (reg == "PITCH_E") { regulator = pitchRegulatorExtern; } 
  else if (reg == "YAW_E") { regulator = yawRegulatorExtern; }
  else if (reg == "ALTITUDE_E") { regulator = altitudeRegulatorExtern; }
  else { return; }
  // obtine valorile constante
  kp = getRegulatorKP(regulator);
  ki = getRegulatorKI(regulator);
  kd = getRegulatorKD(regulator);
  // formeaza json-ul necesar
  String json = "{";
  json += "\"reg\":\"" + reg + "\",";
  json += "\"kp\":\"" + String(kp, 3) + "\",";
  json += "\"ki\":\"" + String(ki, 3) + "\",";
  json += "\"kd\":\"" + String(kd, 3) + "\"";
  json += "}";
  // trimite json-ul prin UDP
  Serial.println(json);
  sendUDP(json);
}
