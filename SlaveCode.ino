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

const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int BASE_SERVO_STEP_ANGLE = 2;
const int FAST_BUTTON_STEP_ANGLE = BASE_SERVO_STEP_ANGLE * 5; // 5x schneller für D12/D14
const int JOYSTICK_BUTTON_STEP_ANGLE = BASE_SERVO_STEP_ANGLE * 4; // Pin 7 über Joystick-Buttons 4x schneller
const unsigned long MOVE_REPEAT_MS = 50;
const int DEFAULT_JOYSTICK_CENTER = 2048;
const int JOYSTICK_DEADZONE = 380;

// Entprellzeit für Reset-Taster
const unsigned long BUTTON_DEBOUNCE_MS = 120;

unsigned long lastButton1Time = 0;
unsigned long lastMoveStepTime = 0;
bool lastResetPressed = false;
int lastSentAngle[8];
int currentServoAngles[8];
int filteredJoy1X = DEFAULT_JOYSTICK_CENTER;
int filteredJoy1Y = DEFAULT_JOYSTICK_CENTER;
int filteredJoy2X = DEFAULT_JOYSTICK_CENTER;
int filteredJoy2Y = DEFAULT_JOYSTICK_CENTER;
int joy1XCenter = DEFAULT_JOYSTICK_CENTER;
int joy1YCenter = DEFAULT_JOYSTICK_CENTER;
int joy2XCenter = DEFAULT_JOYSTICK_CENTER;
int joy2YCenter = DEFAULT_JOYSTICK_CENTER;

// Daten vom Master
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
  bool servo0CcwPressed;
  bool servo0CwPressed;
} struct_message;

struct_message receivedData;

void sendServoAngleToArduino(int servoIndex, int angle) {
  if (servoIndex < 0 || servoIndex > 7) {
    return;
  }

  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // Nur senden, wenn sich der Wert geändert hat
  if (angle == lastSentAngle[servoIndex]) {
    return;
  }

  ArduinoSerial.print(servoIndex);
  ArduinoSerial.print(' ');
  ArduinoSerial.println(angle);
  lastSentAngle[servoIndex] = angle;
  currentServoAngles[servoIndex] = angle;

  Serial.print("-> Arduino CMD: ");
  Serial.print(servoIndex);
  Serial.print(' ');
  Serial.println(angle);
}

void sendResetAllToArduino() {
  for (int servoIndex = 1; servoIndex <= 7; servoIndex++) {
    ArduinoSerial.print(servoIndex);
    ArduinoSerial.println(" 90");
    currentServoAngles[servoIndex] = 90;
    lastSentAngle[servoIndex] = 90;
  }
  Serial.println("Alle Servos RESET -> 90°");
}

int applyAxisFilter(int previousValue, int rawValue) {
  // Einfache Glättung gegen Zittern
  return (previousValue * 3 + rawValue) / 4;
}

void updateAxisCenter(int &centerValue, int filteredValue) {
  // Langsames Nachführen nur in Center-Nähe
  if (abs(filteredValue - centerValue) < 250) {
    centerValue = (centerValue * 31 + filteredValue) / 32;
  }
}

int mapJoystickToStep(int axisValue, int axisCenter) {
  int delta = axisValue - axisCenter;
  int absDelta = abs(delta);
  if (absDelta <= JOYSTICK_DEADZONE) {
    return 0;
  }

  int maxDelta = DEFAULT_JOYSTICK_CENTER - JOYSTICK_DEADZONE;
  int clampedDelta = min(absDelta - JOYSTICK_DEADZONE, maxDelta);
  int step = map(clampedDelta, 0, maxDelta, 1, 10);
  return (delta > 0) ? step : -step;
}

void moveServoWithStep(int servoIndex, int deltaStep) {
  if (deltaStep == 0) {
    return;
  }
  int nextAngle = constrain(currentServoAngles[servoIndex] + deltaStep, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  sendServoAngleToArduino(servoIndex, nextAngle);
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  (void)recv_info;
  if (len != sizeof(receivedData)) {
    return;
  }

  memcpy(&receivedData, incomingData, sizeof(receivedData));

  bool cwPressed = receivedData.rotateCwPressed;     // D12 -> Servo 1 CW
  bool ccwPressed = receivedData.rotateCcwPressed;   // D14 -> Servo 1 CCW

  // Reset nur bei echter Druck-Flanke + Entprellung (D13)
  if (receivedData.resetPressed && !lastResetPressed) {
    if (millis() - lastButton1Time > BUTTON_DEBOUNCE_MS) {
      sendResetAllToArduino();
      lastButton1Time = millis();
    }
  }

  // Servo-Logik:
  // 1) Servo 1 über D12/D14 (5x schneller als vorher)
  // 2) Servo 7 über Joystick-Buttons
  // 3) Servo 0 über D2/D4
  // 4) Servo 2/3/4/5 proportional über Joystick-Achsen
  if (millis() - lastMoveStepTime >= MOVE_REPEAT_MS) {
    // Servo 1 (Shield-Pin 1): D12/D14
    if (cwPressed && !ccwPressed) {
      moveServoWithStep(2, FAST_BUTTON_STEP_ANGLE);
    } else if (ccwPressed && !cwPressed) {
      moveServoWithStep(2, -FAST_BUTTON_STEP_ANGLE);
    }

    // Servo 7: Joystick-Buttons
    if (receivedData.joy1Pressed && !receivedData.joy2Pressed) {
      moveServoWithStep(7, JOYSTICK_BUTTON_STEP_ANGLE);
    } else if (receivedData.joy2Pressed && !receivedData.joy1Pressed) {
      moveServoWithStep(7, -JOYSTICK_BUTTON_STEP_ANGLE);
    }

    // Servo 0: D2/D4
    if (receivedData.servo0CwPressed && !receivedData.servo0CcwPressed) {
      moveServoWithStep(1, BASE_SERVO_STEP_ANGLE);
    } else if (receivedData.servo0CcwPressed && !receivedData.servo0CwPressed) {
      moveServoWithStep(1, -BASE_SERVO_STEP_ANGLE);
    }

    // Joystick 1 Y -> Servo 5 und 6 gleichlaufend (im Arduino: Index 6 steuert Pin 5+6)
    filteredJoy1Y = applyAxisFilter(filteredJoy1Y, receivedData.joy1Y);
    updateAxisCenter(joy1YCenter, filteredJoy1Y);
    int joy1YStep = mapJoystickToStep(filteredJoy1Y, joy1YCenter);
    // Nach unten -> gegen Uhrzeigersinn, nach oben -> im Uhrzeigersinn
    moveServoWithStep(6, -joy1YStep);

    // Joystick 1 X -> Servo 4
    filteredJoy1X = applyAxisFilter(filteredJoy1X, receivedData.joy1X);
    updateAxisCenter(joy1XCenter, filteredJoy1X);
    int joy1XStep = mapJoystickToStep(filteredJoy1X, joy1XCenter);
    // Nach links -> im Uhrzeigersinn, nach rechts -> gegen Uhrzeigersinn
    moveServoWithStep(5, -joy1XStep);

    // Joystick 2 X -> Servo 3
    filteredJoy2X = applyAxisFilter(filteredJoy2X, receivedData.joy2X);
    updateAxisCenter(joy2XCenter, filteredJoy2X);
    int joy2XStep = mapJoystickToStep(filteredJoy2X, joy2XCenter);
    // Nach links -> im Uhrzeigersinn, nach rechts -> gegen Uhrzeigersinn
    moveServoWithStep(4, -joy2XStep);

    // Joystick 2 Y -> Servo 2
    filteredJoy2Y = applyAxisFilter(filteredJoy2Y, receivedData.joy2Y);
    updateAxisCenter(joy2YCenter, filteredJoy2Y);
    int joy2YStep = mapJoystickToStep(filteredJoy2Y, joy2YCenter);
    // Nach oben -> gegen Uhrzeigersinn, nach unten -> im Uhrzeigersinn
    moveServoWithStep(3, joy2YStep);

    lastMoveStepTime = millis();
  }

  lastResetPressed = receivedData.resetPressed;

  Serial.print("CW: ");
  Serial.print(cwPressed);
  Serial.print(" | CCW: ");
  Serial.print(ccwPressed);
  Serial.print(" | RESET: ");
  Serial.print(receivedData.resetPressed);
  Serial.print(" | S1: ");
  Serial.print(currentServoAngles[2]);
  Serial.print(" | S7: ");
  Serial.print(currentServoAngles[7]);
  Serial.print(" | S0: ");
  Serial.println(currentServoAngles[1]);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  for (int i = 0; i < 8; i++) {
    currentServoAngles[i] = 90;
    lastSentAngle[i] = -1;
  }

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
