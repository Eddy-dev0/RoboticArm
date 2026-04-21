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
const int SERVO_STEP_ANGLE = 2;
const unsigned long MOVE_REPEAT_MS = 50;

// Entprellzeit für Reset-Taster
const unsigned long BUTTON_DEBOUNCE_MS = 120;

unsigned long lastButton1Time = 0;
unsigned long lastMoveStepTime = 0;
bool lastResetPressed = false;
int lastSentAngle = -1;
int currentServoAngle = 90;

// Daten vom Master (Joystick-Felder sind aktuell ohne Funktion)
typedef struct struct_message {
  int joy1X;
  int joy1Y;
  bool joy1Pressed;
  int joy2X;
  int joy2Y;
  bool joy2Pressed;

  bool rotateCwPressed;
  bool rotateCcwPressed;
  bool resetPressed;
} struct_message;

struct_message receivedData;

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

void sendResetAllToArduino() {
  for (int servoIndex = 1; servoIndex <= 7; servoIndex++) {
    ArduinoSerial.print(servoIndex);
    ArduinoSerial.println(" 90");
  }

  currentServoAngle = 90;
  lastSentAngle = -1; // erlaube erneutes Senden des gleichen Winkels nach Komplett-Reset
  sendServoAngleToArduino(currentServoAngle);
  Serial.println("Alle Servos RESET -> 90°");
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  (void)recv_info;
  if (len != sizeof(receivedData)) {
    return;
  }

  memcpy(&receivedData, incomingData, sizeof(receivedData));

  bool cwPressed = receivedData.rotateCwPressed;
  bool ccwPressed = receivedData.rotateCcwPressed;

  // Reset nur bei echter Druck-Flanke + Entprellung (D13)
  if (receivedData.resetPressed && !lastResetPressed) {
    if (millis() - lastButton1Time > BUTTON_DEBOUNCE_MS) {
      sendResetAllToArduino();
      lastButton1Time = millis();
    }
  }

  // Drehung von Servo 7 nur mit D12 / D14 (gehalten = wiederholte Schritte)
  if (millis() - lastMoveStepTime >= MOVE_REPEAT_MS) {
    if (cwPressed && !ccwPressed) {
      currentServoAngle = constrain(currentServoAngle + SERVO_STEP_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      sendServoAngleToArduino(currentServoAngle);
      lastMoveStepTime = millis();
    } else if (ccwPressed && !cwPressed) {
      currentServoAngle = constrain(currentServoAngle - SERVO_STEP_ANGLE, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      sendServoAngleToArduino(currentServoAngle);
      lastMoveStepTime = millis();
    }
  }

  lastResetPressed = receivedData.resetPressed;

  Serial.print("CW: ");
  Serial.print(cwPressed);
  Serial.print(" | CCW: ");
  Serial.print(ccwPressed);
  Serial.print(" | RESET: ");
  Serial.print(receivedData.resetPressed);
  Serial.print(" | Servo7: ");
  Serial.println(currentServoAngle);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  ArduinoSerial.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX2_PIN, ARDUINO_TX2_PIN);

  // Arduino in manuellen Modus setzen (direkte Servo-Kommandos)
  ArduinoSerial.println("MODE 0");
  sendResetAllToArduino();

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
  // Keine zyklische Rücksetzung:
  // Servo bleibt auf letzter Position, bis Reset-Taster (D13) gedrückt wird.
}
