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
const int BUTTON_CW_PIN = 12;
const int BUTTON_RESET_PIN = 13;
const int BUTTON_CCW_PIN = 14;
const int BUTTON_SERVO0_CW_PIN = 22;
const int BUTTON_SERVO0_CCW_PIN = 23;
const bool BUTTON_SERVO0_CCW_ACTIVE_LOW = true;
const bool BUTTON_SERVO0_CW_ACTIVE_LOW = true;

// Neue PWM-LEDs (Empfindlichkeitsanzeige)
// Reihenfolge wichtig: LED1 = kleinste Stufe, LED4 = höchste Stufe
const int LED1_PIN = 18;
const int LED2_PIN = 19;
const int LED3_PIN = 21;
const int LED4_PIN = 25;

// Rotary Encoder
const int ENC_CLK_PIN = 4;
const int ENC_DT_PIN = 16;
const int ENC_SW_PIN = 17;

// Separate Verbindungs-Status-LED (wie früher: an = verbunden, blinken = suche)
const int STATUS_LED_PIN = 2;
const unsigned long STATUS_LED_BLINK_MS = 300;
const unsigned long CONNECTION_TIMEOUT_MS = 1000;

const int START_SENSITIVITY = 1;
const int MAX_SENSITIVITY = 32;
const int MIN_PWM = 5;
const int MAX_PWM = 127;

uint8_t receiverMAC[] = {0x80, 0xF3, 0xDA, 0xBA, 0xA3, 0xF8};

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
  int sensitivityLevel;
} struct_message;

struct_message dataToSend;
int sensitivityLevel = START_SENSITIVITY;
int lastEncoderClk = HIGH;
unsigned long lastEncoderStepMs = 0;
bool slaveConnected = false;
unsigned long lastSuccessSendMs = 0;
unsigned long lastBlinkToggleMs = 0;
bool blinkLedState = false;

bool readButtonState(int pin, bool activeLow) {
  int raw = digitalRead(pin);
  return activeLow ? (raw == LOW) : (raw == HIGH);
}

void updateSensitivityLeds() {
  int values[4] = {0, 0, 0, 0};
  int remaining = sensitivityLevel;

  for (int i = 0; i < 4; i++) {
    if (remaining >= 8) {
      values[i] = MAX_PWM;
      remaining -= 8;
    } else if (remaining > 0) {
      values[i] = map(remaining, 1, 8, MIN_PWM, MAX_PWM);
      remaining = 0;
    }
  }

  ledcWrite(LED1_PIN, values[0]);
  ledcWrite(LED2_PIN, values[1]);
  ledcWrite(LED3_PIN, values[2]);
  ledcWrite(LED4_PIN, values[3]);
}


void runStartupSensitivityLedAnimation() {
  const int stepDelayMs = 60;

  for (int i = 0; i < 4; i++) {
    ledcWrite(LED1_PIN, 0);
    ledcWrite(LED2_PIN, 0);
    ledcWrite(LED3_PIN, 0);
    ledcWrite(LED4_PIN, 0);
    if (i == 0) ledcWrite(LED1_PIN, MAX_PWM);
    if (i == 1) ledcWrite(LED2_PIN, MAX_PWM);
    if (i == 2) ledcWrite(LED3_PIN, MAX_PWM);
    if (i == 3) ledcWrite(LED4_PIN, MAX_PWM);
    delay(stepDelayMs);
  }

  for (int i = 3; i >= 0; i--) {
    ledcWrite(LED1_PIN, 0);
    ledcWrite(LED2_PIN, 0);
    ledcWrite(LED3_PIN, 0);
    ledcWrite(LED4_PIN, 0);
    if (i == 0) ledcWrite(LED1_PIN, MAX_PWM);
    if (i == 1) ledcWrite(LED2_PIN, MAX_PWM);
    if (i == 2) ledcWrite(LED3_PIN, MAX_PWM);
    if (i == 3) ledcWrite(LED4_PIN, MAX_PWM);
    delay(stepDelayMs);
  }

  sensitivityLevel = START_SENSITIVITY;
  updateSensitivityLeds();
}


void handleEncoder() {
  const unsigned long ENCODER_STEP_DEBOUNCE_MS = 2;
  int currentClk = digitalRead(ENC_CLK_PIN);
  unsigned long now = millis();

  // nur auf fallende Flanke reagieren (robuster gegen Prellen und Fehlschritte)
  if (lastEncoderClk == HIGH && currentClk == LOW && (now - lastEncoderStepMs) >= ENCODER_STEP_DEBOUNCE_MS) {
    if (digitalRead(ENC_DT_PIN) == HIGH) {
      sensitivityLevel++;
    } else {
      sensitivityLevel--;
    }

    sensitivityLevel = constrain(sensitivityLevel, START_SENSITIVITY, MAX_SENSITIVITY);
    updateSensitivityLeds();
    Serial.print("Encoder -> Sensitivity: ");
    Serial.println(sensitivityLevel);
    lastEncoderStepMs = now;
  }
  lastEncoderClk = currentClk;

  if (digitalRead(ENC_SW_PIN) == LOW) {
    sensitivityLevel = START_SENSITIVITY;
    updateSensitivityLeds();
    Serial.println("Encoder button -> Sensitivity reset");
    delay(250);
  }
}

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSuccessSendMs = millis();
  }
}

void updateStatusLed() {
  unsigned long now = millis();
  slaveConnected = (now - lastSuccessSendMs) <= CONNECTION_TIMEOUT_MS;

  if (slaveConnected) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }

  if (now - lastBlinkToggleMs >= STATUS_LED_BLINK_MS) {
    blinkLedState = !blinkLedState;
    lastBlinkToggleMs = now;
  }
  digitalWrite(STATUS_LED_PIN, blinkLedState ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON_CW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RESET_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CCW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SERVO0_CCW_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SERVO0_CW_PIN, INPUT_PULLUP);
  pinMode(joy1SW, INPUT_PULLUP);
  pinMode(joy2SW, INPUT_PULLUP);

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  ledcAttach(LED1_PIN, 5000, 8);
  ledcAttach(LED2_PIN, 5000, 8);
  ledcAttach(LED3_PIN, 5000, 8);
  ledcAttach(LED4_PIN, 5000, 8);

  lastEncoderClk = digitalRead(ENC_CLK_PIN);
  runStartupSensitivityLedAnimation();

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
  handleEncoder();

  dataToSend.joy1X = analogRead(joy1X);
  dataToSend.joy1Y = analogRead(joy1Y);
  dataToSend.joy1Pressed = (digitalRead(joy1SW) == LOW);
  dataToSend.joy2X = analogRead(joy2X);
  dataToSend.joy2Y = analogRead(joy2Y);
  dataToSend.joy2Pressed = (digitalRead(joy2SW) == LOW);

  dataToSend.rotateCwPressed = (digitalRead(BUTTON_CW_PIN) == LOW);
  dataToSend.rotateCcwPressed = (digitalRead(BUTTON_CCW_PIN) == LOW);
  dataToSend.resetPressed = (digitalRead(BUTTON_RESET_PIN) == LOW);
  dataToSend.servo0CcwPressed = readButtonState(BUTTON_SERVO0_CCW_PIN, BUTTON_SERVO0_CCW_ACTIVE_LOW);
  dataToSend.servo0CwPressed = readButtonState(BUTTON_SERVO0_CW_PIN, BUTTON_SERVO0_CW_ACTIVE_LOW);
  dataToSend.sensitivityLevel = sensitivityLevel;

  esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  updateStatusLed();

  delay(50);
}
