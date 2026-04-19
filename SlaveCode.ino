#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

Servo servo1;
Servo servo2;

const int servoPin1 = 27;
const int servoPin2 = 26;

// STOP-Werte anpassen!
// Wenn ein Servo noch kriecht:
// 88, 89, 90, 91, 92 testen
const int SERVO1_STOP = 89;
const int SERVO2_STOP = 89;

// Deadzone um Mittelstellung des Joysticks
const int DEADZONE = 300;

// Timeout: wenn keine Daten mehr kommen -> STOP
const unsigned long SIGNAL_TIMEOUT = 500;

// Entprellzeit für Reset-Taster
const unsigned long BUTTON_DEBOUNCE_MS = 120;

unsigned long lastReceiveTime = 0;
unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;

bool lastJoy1Pressed = false;
bool lastJoy2Pressed = false;

typedef struct struct_message {
  int joy1X;
  int joy1Y;
  bool joy1Pressed;

  int joy2X;
  int joy2Y;
  bool joy2Pressed;
} struct_message;

struct_message receivedData;

// Joystickwert auf Servo-Wert für 360°-Servo umrechnen
int joystickToServoValue(int joyValue, int stopValue) {
  const int center = 2048;
  int diff = joyValue - center;

  if (abs(diff) < DEADZONE) {
    return stopValue;
  }

  int offset = map(diff, -2048, 2047, -80, 80);
  int out = stopValue + offset;

  return constrain(out, 0, 180);
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(receivedData)) {
    return;
  }

  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();

  int servoValue1 = joystickToServoValue(receivedData.joy1X, SERVO1_STOP);
  int servoValue2 = joystickToServoValue(receivedData.joy2X, SERVO2_STOP);

  // Servo 1 normal steuern
  servo1.write(servoValue1);

  // Servo 2 normal steuern
  servo2.write(servoValue2);

  // Reset nur bei echter Druck-Flanke + Entprellung
  if (receivedData.joy1Pressed && !lastJoy1Pressed) {
    if (millis() - lastButton1Time > BUTTON_DEBOUNCE_MS) {
      servo1.write(SERVO1_STOP);
      lastButton1Time = millis();
      Serial.println("Servo 1 RESET/STOP");
    }
  }

  if (receivedData.joy2Pressed && !lastJoy2Pressed) {
    if (millis() - lastButton2Time > BUTTON_DEBOUNCE_MS) {
      servo2.write(SERVO2_STOP);
      lastButton2Time = millis();
      Serial.println("Servo 2 RESET/STOP");
    }
  }

  lastJoy1Pressed = receivedData.joy1Pressed;
  lastJoy2Pressed = receivedData.joy2Pressed;

  Serial.print("J1X: ");
  Serial.print(receivedData.joy1X);
  Serial.print(" | SW1: ");
  Serial.print(receivedData.joy1Pressed);
  Serial.print(" | Servo1: ");
  Serial.print(servoValue1);

  Serial.print(" || J2X: ");
  Serial.print(receivedData.joy2X);
  Serial.print(" | SW2: ");
  Serial.print(receivedData.joy2Pressed);
  Serial.print(" | Servo2: ");
  Serial.println(servoValue2);
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(300);

  Serial.print("ESP2 MAC-Adresse: ");
  Serial.println(WiFi.macAddress());

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);

  servo1.attach(servoPin1, 500, 2400);
  servo2.attach(servoPin2, 500, 2400);

  // Anfangszustand = STOP
  servo1.write(SERVO1_STOP);
  servo2.write(SERVO2_STOP);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW bereit");
}

void loop() {
  // Falls Verbindung weg -> beide stoppen
  if (millis() - lastReceiveTime > SIGNAL_TIMEOUT) {
    servo1.write(SERVO1_STOP);
    servo2.write(SERVO2_STOP);
  }
}