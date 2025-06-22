#ifndef UDP_HANDLER_H
#define UDP_HANDLER_H
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "Config.h"
// ajuta la receptarea trimise
extern StaticJsonDocument<200> jsonDoc;
// creeaza conexiunea wifi
void setupUDP();
// trimite parametrul 'message' prin UDP
void sendUDP(const String& message);
// asculta pe port, retruneaza adevarat daca un mesaj a fost receptionat
// salveaza mesajul in variabila 'jsonDoc'
bool readUDP();
#endif