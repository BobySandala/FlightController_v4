#include "BatteryLevelReader.h"
// declararea variabilelor globale
TaskHandle_t batteryLevelTaskHandle; // handle
// bucla infinita
void batteryLevelTask(void* pvParameters) {
  while (true) {
    readBatteryLevel();
    vTaskDelay(pdMS_TO_TICKS(10000)); // 10s
  }
}
// citeste nivelul unei celule si calculeaza nivelul total
void readBatteryLevel() {
  int adcValue = analogRead(BATTERY_PIN); // citirea valorii analogice a unei celule
  float rawVoltage = adcValue * (3.3 / 4095.0); // conversia din adc in volti
  float correctedVoltage = rawVoltage * (1.9 / 1.7);  // ajustare bazata pe valoarea citita de multimetru
  // calcul voltaj total baterie Resistor1, Resistor2 valorile rezistorului din divizor, inmultireacu 3 deoarece sunt 3 celule
  float batteryVoltage = correctedVoltage * ((Resistor1 + Resistor2) / Resistor2) * 3; 
  // pin neconectat
  if (rawVoltage == 0.0)  {  } 
  // nivel scazut
  else if (batteryVoltage < 11.8) {  }
  // nivel normal
  else {  }
}
// initializeaza led-ul si creaza task-ul
void batteryReaderInit() {
  
  xTaskCreatePinnedToCore(
    batteryLevelTask,         // functie
    "BatteryLevelTask",       // nume
    4096,                     // dimensiune
    NULL,                     // parametrii
    1,                        // prioritate
    &batteryLevelTaskHandle,  // handle
    0                         // core 0
  );
}