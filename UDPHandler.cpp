#include "UdpHandler.h"
// definirea variabilelor globale
WiFiUDP udp; // clasa care contine metodele UDP
StaticJsonDocument<200> jsonDoc; // obiect Json pentru deserializare
TaskHandle_t WiFiTaskHandle;  // Task Handle pentru Wi-Fi
// Task paralel, stabileste conxiunea wifi la reteauadefinita infisierul config
void WiFiTask(void* pvParameters) {
  WiFi.mode(WIFI_STA); // mod client
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // conectarea la reteaua specificata
  int attempts = 0; // numar actual de incercari
  const int maxAttempts = 20;  // numar maxim de incercari
  // incearca conectarea o data pe secunda
  while (WiFi.status() != WL_CONNECTED && attempts++ < maxAttempts) { delay(1000); }
  // daca este conectat cu succes porneste comunicatia udp pe portul specificat
  if (WiFi.status() == WL_CONNECTED) {
    udp.begin(UDP_PORT);
    Serial.println("Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("UDP Port: ");
    Serial.println(UDP_PORT);
  }
  vTaskDelete(NULL);  // distruge taskul dupa conectare sau timeout
}

// creeaza task-ul paralel pentru conectarea wifi
void setupUDP() {
  xTaskCreatePinnedToCore(
    WiFiTask,         // functia
    "WiFiTask",       // numele
    4096,             // dimensiunea
    NULL,             // parametrii
    1,                // prioritatea
    &WiFiTaskHandle,  // handle-ul
    0                 // Core 1
  );
}

void sendUDP(const String& message) {
  // eroare in caz de mesaj gol
  if (message.length() == 0) { return; }
  // eroarea in caz de inexistenta conexiunii wifi
  if (!udp.beginPacket(UDP_ADDRESS, UDP_PORT)) { return; }
  // adauga date pentru trimitere, returneaza numarul de bytes adaugati
  // se face cast la tipul de date char* (uint8_t*)
  int bytesWritten = udp.write(reinterpret_cast<const uint8_t*>(message.c_str()), message.length());
  // trimite pachetul
  bool endPack = udp.endPacket();
}

bool readUDP() {
  // verifica daca exista pachet de citit
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    // citeste pachetul in variabila incomingPachet
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    // adauga terminatorul de sir
    if (len > 0) incomingPacket[len] = '\0';
    // mesajul primit este un json care trebuie deserializat
    DeserializationError error = deserializeJson(jsonDoc, incomingPacket);
    // returneaza true daca un exista erori si false altfel
    if (!error) { return true; } 
    else { return false; }
  }
  return false;
}
