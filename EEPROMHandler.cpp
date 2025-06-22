#include "EEPROMHandler.h"

Preferences prefs; // definirea variabilei globale
// salveaza valoarea 'value' in EEPROM si ii atribuie tag-ul 'key'
void saveFloat(String key, float value) {
  prefs.begin("storage", false);  // false = read/write
  prefs.putFloat(key.c_str(), value); // scriere
  prefs.end(); // finalizare
  Serial.print("valoare salvata   ");
  Serial.println(loadFloat(key, 0));
}
// returneaza valoarea stocata cu tag-ul 'key' sau o valoare implicita
float loadFloat(String key, float defaultVal = 0.0f) {
  prefs.begin("storage", true);  // true = read-only
  float val = prefs.getFloat(key.c_str(), defaultVal); // citirea
  prefs.end(); // finalizare
  return val;
}