#include <WiFi.h>
#include <esp_now.h>

// Joysticks + Joystick-Buttons
const int joy1X  = 34;
const int joy1Y  = 35;
const int joy1SW = 27;
const int joy2X  = 32;
const int joy2Y  = 33;
const int joy2SW = 26;

// Taster-Pins (Master)
// D12 = im Uhrzeigersinn, D14 = gegen Uhrzeigersinn, D13 = Reset
const int BUTTON_CW_PIN = 12;
const int BUTTON_RESET_PIN = 13;
const int BUTTON_CCW_PIN = 14;
// Zusätzliche Taster für Servo an Shield-Pin 0
const int BUTTON_SERVO0_CCW_PIN = 2;
const int BUTTON_SERVO0_CW_PIN = 4;

// MAC-Adresse vom EMPFÄNGER-ESP32 hier eintragen!
uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0xBA, 0xA3, 0xF8};

// Datenstruktur
typedef struct struct_message {
  // Joystick-Felder bleiben im Protokoll, werden aber aktuell nicht verwendet.
  int joy1X;
  int joy1Y;
  bool joy1Pressed;
  int joy2X;
  int joy2Y;
  bool joy2Pressed;

  bool rotateCwPressed;
  bool rotateCcwPressed;
  bool resetPressed;
  bool servo0CcwPressed;
  bool servo0CwPressed;
} struct_message;

struct_message dataToSend;

// Callback nach dem Senden
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Sendestatus: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehler");
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_CW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RESET_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CCW_PIN, INPUT_PULLUP);
  // D2/D4 sind separat verdrahtet (aktiv HIGH)
  pinMode(BUTTON_SERVO0_CCW_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_SERVO0_CW_PIN, INPUT_PULLDOWN);
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
  // Joystickwerte werden kontinuierlich an den Slave gesendet.
  dataToSend.joy1X = analogRead(joy1X);
  dataToSend.joy1Y = analogRead(joy1Y);
  dataToSend.joy1Pressed = (digitalRead(joy1SW) == LOW);
  dataToSend.joy2X = analogRead(joy2X);
  dataToSend.joy2Y = analogRead(joy2Y);
  dataToSend.joy2Pressed = (digitalRead(joy2SW) == LOW);

  // gedrückt = LOW (wegen INPUT_PULLUP)
  dataToSend.rotateCwPressed = (digitalRead(BUTTON_CW_PIN) == LOW);
  dataToSend.rotateCcwPressed = (digitalRead(BUTTON_CCW_PIN) == LOW);
  dataToSend.resetPressed = (digitalRead(BUTTON_RESET_PIN) == LOW);
  // D2/D4 aktiv HIGH
  dataToSend.servo0CcwPressed = (digitalRead(BUTTON_SERVO0_CCW_PIN) == HIGH);
  dataToSend.servo0CwPressed = (digitalRead(BUTTON_SERVO0_CW_PIN) == HIGH);

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  if (result != ESP_OK) {
    Serial.println("Fehler beim Senden");
  }

  Serial.print("CW: ");
  Serial.print(dataToSend.rotateCwPressed);
  Serial.print(" | CCW: ");
  Serial.print(dataToSend.rotateCcwPressed);
  Serial.print(" | RESET: ");
  Serial.print(dataToSend.resetPressed);
  Serial.print(" | S0_CCW(D2): ");
  Serial.print(dataToSend.servo0CcwPressed);
  Serial.print(" | S0_CW(D4): ");
  Serial.print(dataToSend.servo0CwPressed);
  Serial.print(" | J1X: ");
  Serial.print(dataToSend.joy1X);
  Serial.print(" | J2X: ");
  Serial.println(dataToSend.joy2X);

  delay(50);
}
