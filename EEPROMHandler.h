#ifndef EEPROM_HANDLER
#define EEPROM_HANDLER
#include <Preferences.h>

extern Preferences prefs; // instanta a clasei definita de librarie
void saveFloat(String key, float value); // salveaza in EEPROM
float loadFloat(String key, float defaultVal); // citeste din EEPROM
#endif