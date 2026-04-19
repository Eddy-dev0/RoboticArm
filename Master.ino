#include <WiFi.h>
#include <esp_now.h>

// Joystick Pins
const int joy1X  = 34;
const int joy1Y  = 35;
const int joy1SW = 27;

const int joy2X  = 32;
const int joy2Y  = 33;
const int joy2SW = 26;

// MAC-Adresse vom EMPFÄNGER-ESP32 hier eintragen!
uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0xBA, 0xA3, 0xF8};

// Datenstruktur
typedef struct struct_message {
  int joy1X;
  int joy1Y;
  bool joy1Pressed;

  int joy2X;
  int joy2Y;
  bool joy2Pressed;
} struct_message;

struct_message dataToSend;

// Callback nach dem Senden
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Sendestatus: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehler");
}

void setup() {
  Serial.begin(115200);

  pinMode(joy1SW, INPUT_PULLUP);
  pinMode(joy2SW, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("ESP1 MAC-Adresse: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler beim Hinzufügen des Peers");
    return;
  }
}

void loop() {
  dataToSend.joy1X = analogRead(joy1X);
  dataToSend.joy1Y = analogRead(joy1Y);
  dataToSend.joy1Pressed = (digitalRead(joy1SW) == LOW);  // gedrückt = LOW

  dataToSend.joy2X = analogRead(joy2X);
  dataToSend.joy2Y = analogRead(joy2Y);
  dataToSend.joy2Pressed = (digitalRead(joy2SW) == LOW);  // gedrückt = LOW

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  if (result != ESP_OK) {
    Serial.println("Fehler beim Senden");
  }

  Serial.print("J1X: ");
  Serial.print(dataToSend.joy1X);
  Serial.print("  J1SW: ");
  Serial.print(dataToSend.joy1Pressed);

  Serial.print("   |   J2X: ");
  Serial.print(dataToSend.joy2X);
  Serial.print("  J2SW: ");
  Serial.println(dataToSend.joy2Pressed);

  delay(50);
}