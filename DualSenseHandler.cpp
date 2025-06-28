#include "DualSenseHandler.h"
#include <dual-sense-controller-types.h>
#include <DualSenseControllerBt.h>

// Global variable definition
CONTROLLER_INPUT ControllerInputData;

// Configuration constants
const bool PAIR_CONTROLLER_WHILE_SETUP = false;
const bool DEBUG_PRINT = false;

void printValueIfDebugPrint(String s, int16_t value);
void updateControllerInputStruct();

// Raw controller values (to be mapped to your struct)
int16_t rawLeftStickX = 0;
int16_t rawLeftStickY = 0;
int16_t rawRightStickX = 0;
int16_t rawL2 = 0;
int16_t rawR2 = 0;

void initDualSenseController() {
    Serial.begin(115200);
    delay(500);
    Serial.println(F("DualSense Controller Setup"));
    
    // Initialize controller input struct
    ControllerInputData.StickX = 0.0;
    ControllerInputData.StickY = 0.0;
    ControllerInputData.StickRX = 0.0;
    ControllerInputData.Throttle = 0.0;
    ControllerInputData.Brake = 0.0;
    ControllerInputData.kill = false;
    ControllerInputData.dpadUp = false;
    ControllerInputData.dpadDown = false;
    ControllerInputData.dpadLeft = false;
    ControllerInputData.dpadRight = false;
    
    ps5.begin(MAC_ADDRESS); //replace with MAC address of your controller

    xTaskCreatePinnedToCore(
        updateDualSenseController, // functia
        "DS4",                     // numele
        4096,                      // dimensiunea
        NULL,                      // parametrii
        1,                         // prioritatea
        NULL,                      // handle-ul
        0                          // core 1
    );
}

void updateDualSenseController(void* params) {
    while (1)
    {
        if (ps5.isConnected()) { updateControllerInputStruct(); printControllerData(); }
        vTaskDelay(20);
    }
}


void updateControllerInputStruct() {
    // Map analog sticks from raw values (-32768 to 32767) to float (-1.0 to 1.0)
    ControllerInputData.StickX = ps5.LStickX();
    ControllerInputData.StickY = ps5.LStickY();
    ControllerInputData.StickRX = ps5.RStickX();
    
    // Map triggers from raw values (0 to 255) to float (0.0 to 1.0)
    ControllerInputData.Throttle = ps5.R2();  // R2 as throttle
    ControllerInputData.Brake = ps5.L2();     // L2 as brake
    
    // Apply deadzone to sticks (optional - removes small unwanted movements)
    const float deadzone = 0.1f;
    if (abs(ControllerInputData.StickX) < deadzone) ControllerInputData.StickX = 0.0f;
    if (abs(ControllerInputData.StickY) < deadzone) ControllerInputData.StickY = 0.0f;
    if (abs(ControllerInputData.StickRX) < deadzone) ControllerInputData.StickRX = 0.0f;
    
    ControllerInputData.dpadUp = ps5.Up();
    ControllerInputData.dpadDown = ps5.Down();
    ControllerInputData.dpadLeft = ps5.Right();
    ControllerInputData.dpadRight = ps5.Left();

    ControllerInputData.kill = ps5.Cross();
}

void printControllerData() {
    if (!DEBUG_PRINT) return;
    
    Serial.println("=== CONTROLLER INPUT DATA ===");
    Serial.print("StickX: "); Serial.print(ControllerInputData.StickX, 3);
    Serial.print(" StickY: "); Serial.print(ControllerInputData.StickY, 3);
    Serial.print(" StickRX: "); Serial.print(ControllerInputData.StickRX, 3);
    Serial.print(" Throttle: "); Serial.print(ControllerInputData.Throttle, 3);
    Serial.print(" Brake: "); Serial.print(ControllerInputData.Brake, 3);
    Serial.print(" Kill: "); Serial.print(ControllerInputData.kill ? "ON" : "OFF");
    Serial.print(" DPad: ");
    if (ControllerInputData.dpadUp) Serial.print("UP ");
    if (ControllerInputData.dpadDown) Serial.print("DOWN ");
    if (ControllerInputData.dpadLeft) Serial.print("LEFT ");
    if (ControllerInputData.dpadRight) Serial.print("RIGHT ");
    Serial.println();
}

void printValueIfDebugPrint(String s, int16_t value) {
    if (!DEBUG_PRINT) return;
    Serial.print(s);
    Serial.print(": ");
    Serial.println(value);
}