#ifndef BATTERY_LEVEL_READER
#define BATTERY_LEVEL_READER

#include "Config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern TaskHandle_t batteryLevelTaskHandle; // handle
// inițializarea led-ului si crearea task-ului
void batteryReaderInit();
// citirea nivelului
void readBatteryLevel();
// task-ul bucla infinita
void batteryLevelTask(void* pvParameters);
#endif