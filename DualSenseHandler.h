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
// structura globala pentru stocarea input-ului
extern CONTROLLER_INPUT ControllerInputData;
void initDualSenseController(); // functia de initializare 
void updateDualSenseController(void* params); // task-ul paralel
void printControllerData(); // folosit pentru depanare
extern const bool DEBUG_PRINT; // true / false pentru print

#endif // DUALSENSE_HANDLER_H