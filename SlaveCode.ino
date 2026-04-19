#include <WiFi.h>
#include <esp_now.h>

// ======================
// UART Bridge to Arduino
// ======================
// Verkabelung:
// ESP32 TX2 (GPIO17) -> Arduino RX
// ESP32 GND           -> Arduino GND
// Baudrate muss mit Arduino-Sketch übereinstimmen (9600)
HardwareSerial ArduinoSerial(2);
const int ARDUINO_RX2_PIN = 16; // optional, nicht verkabelt
const int ARDUINO_TX2_PIN = 17; // an Arduino RX
const int ARDUINO_BAUD = 9600;

// ======================
// Demo: nur Servo 7
// ======================
// Im Arduino-Code entspricht "7" dem letzten Servo-Index
// (servoChannels[6] = 7 auf dem PCA9685 Shield)
const int TARGET_SERVO_INDEX = 7;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;

// Deadzone um Mittelstellung des Joysticks
const int DEADZONE = 300;

// Timeout: wenn keine Daten mehr kommen -> Servo in Mittelstellung
const unsigned long SIGNAL_TIMEOUT = 500;

// Entprellzeit für Reset-Taster
const unsigned long BUTTON_DEBOUNCE_MS = 120;

unsigned long lastReceiveTime = 0;
unsigned long lastButton1Time = 0;
bool lastJoy1Pressed = false;
int lastSentAngle = -1;

// Joystick-Daten vom Master
typedef struct struct_message {
  int joy1X;
  int joy1Y;
  bool joy1Pressed;

  int joy2X;
  int joy2Y;
  bool joy2Pressed;
} struct_message;

struct_message receivedData;

int joystickToAngle(int joyValue) {
  const int center = 2048;
  int diff = joyValue - center;

  if (abs(diff) < DEADZONE) {
    return 90;
  }

  return constrain(map(joyValue, 0, 4095, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE), SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

void sendServoAngleToArduino(int angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // Nur senden, wenn sich der Wert geändert hat
  if (angle == lastSentAngle) {
    return;
  }

  ArduinoSerial.print(TARGET_SERVO_INDEX);
  ArduinoSerial.print(' ');
  ArduinoSerial.println(angle);
  lastSentAngle = angle;

  Serial.print("-> Arduino CMD: ");
  Serial.print(TARGET_SERVO_INDEX);
  Serial.print(' ');
  Serial.println(angle);
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(receivedData)) {
    return;
  }

  memcpy(&receivedData, incomingData, sizeof(receivedData));
  lastReceiveTime = millis();

  int servoAngle = joystickToAngle(receivedData.joy1X);
  sendServoAngleToArduino(servoAngle);

  // Reset nur bei echter Druck-Flanke + Entprellung
  if (receivedData.joy1Pressed && !lastJoy1Pressed) {
    if (millis() - lastButton1Time > BUTTON_DEBOUNCE_MS) {
      sendServoAngleToArduino(90);
      lastButton1Time = millis();
      Serial.println("Servo 7 RESET -> 90°");
    }
  }

  lastJoy1Pressed = receivedData.joy1Pressed;

  Serial.print("J1X: ");
  Serial.print(receivedData.joy1X);
  Serial.print(" | SW1: ");
  Serial.print(receivedData.joy1Pressed);
  Serial.print(" | Servo7: ");
  Serial.println(servoAngle);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  ArduinoSerial.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX2_PIN, ARDUINO_TX2_PIN);

  // Arduino in manuellen Modus setzen (direkte Servo-Kommandos)
  ArduinoSerial.println("MODE 0");
  sendServoAngleToArduino(90);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(300);

  Serial.print("ESP2 MAC-Adresse: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW bereit");
}

void loop() {
  // Falls Verbindung weg -> Servo in Mittelstellung
  if (millis() - lastReceiveTime > SIGNAL_TIMEOUT) {
    sendServoAngleToArduino(90);
  }
}
