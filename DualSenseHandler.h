#ifndef DUALSENSE_HANDLER_H
#define DUALSENSE_HANDLER_H

#include <Arduino.h>
#include <ps5Controller.h>
#include "Config.h"

// Controller input structure
struct CONTROLLER_INPUT {
  float StickX, StickY, StickRX, Throttle, Brake;
  bool kill;       // Cross button (X button)
  // D-pad
  bool dpadUp;
  bool dpadDown;
  bool dpadLeft;
  bool dpadRight;
};

// Global variable declaration
extern CONTROLLER_INPUT ControllerInputData;

// Public function declarations
void initDualSenseController();
void updateDualSenseController(void* params);
void printControllerData();

// Configuration constants
extern const bool PAIR_CONTROLLER_WHILE_SETUP;
extern const bool DEBUG_PRINT;

#endif // DUALSENSE_HANDLER_H