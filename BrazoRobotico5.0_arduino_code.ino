#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Constantes
const int TOTAL_SERVOS = 7;
const int MIN_SERVO_VALUE = 0;
const int MAX_SERVO_VALUE = 180;
const int MIN_SPEED_VALUE = 0;
const int MAX_SPEED_VALUE = 100;
const int NO_SMOOTH_THRESHOLD = 50;

// Mapeo de servos a canales del shield
const int servoChannels[TOTAL_SERVOS] = {0, 1, 2, 3, 4, 5, 7}; // Garra, Muñeca1, Muñeca2, BrazoSup, BrazoInf, Antebrazo, Base
const int LINKED_SERVO_CHANNEL = 6; // folgt Servo auf Channel 5 gleichlaufend

// Rangos de pulso para los servos (500-2500µs)
const int SERVO_MIN_PULSE = 500;
const int SERVO_MAX_PULSE = 2500;
const int PULSE_FREQ = 50;
const bool LINKED_SERVO_INVERT_DIRECTION = true;

// Variables de control
float currentPositions[TOTAL_SERVOS];
float targetPositions[TOTAL_SERVOS];
int currentSpeed = 50;
bool autoMode = false;
bool isMoving = false;
int receivedPositions = 0;
String inputBuffer = "";
bool inputComplete = false;
int lastPulseByChannel[16];

// Valores ajustables para control de velocidad
const float MAX_SMOOTH_FACTOR = 0.05;
const int MIN_DELAY = 10;
const int MAX_DELAY = 50;
const float THRESHOLD = 0.5;

int angleToPulse(float angle) {
  int pulse = map(round(angle), 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  return map(pulse, 0, 20000, 0, 4096);
}

float getSmoothFactor() {
  // Si la velocidad es menor o igual a 50, no hay suavizado
  if (currentSpeed <= NO_SMOOTH_THRESHOLD) {
    return 1.0; // Factor que elimina el suavizado
  }
  // Para velocidades mayores a 50, el suavizado es proporcional a la velocidad
  // Cuanto menor sea la velocidad, más rápido será el suavizado
  float speedRatio = (currentSpeed - NO_SMOOTH_THRESHOLD) / (float)(MAX_SPEED_VALUE - NO_SMOOTH_THRESHOLD);
  float adjustedSmoothFactor = MAX_SMOOTH_FACTOR * (1 + (1 - speedRatio));
  return adjustedSmoothFactor;
}

int getMovementDelay() {
  return map(currentSpeed, 0, 100, MAX_DELAY, MIN_DELAY);
}

void setServoAngleIfChanged(int channel, float angle) {
  int pulse = angleToPulse(angle);
  if (lastPulseByChannel[channel] != pulse) {
    pwm.setPWM(channel, 0, pulse);
    lastPulseByChannel[channel] = pulse;
  }
}

float mapAngleForLinkedServo(float sourceAngle) {
  if (LINKED_SERVO_INVERT_DIRECTION) {
    return MAX_SERVO_VALUE - sourceAngle;
  }
  return sourceAngle;
}

void setLinkedServoFromSource(float sourceAngle) {
  float linkedAngle = constrain(mapAngleForLinkedServo(sourceAngle), MIN_SERVO_VALUE, MAX_SERVO_VALUE);
  setServoAngleIfChanged(LINKED_SERVO_CHANNEL, linkedAngle);
}

void setup() {
  Serial.begin(9600);
  inputBuffer.reserve(200);

  pwm.begin();
  pwm.setPWMFreq(PULSE_FREQ);
  Wire.setClock(400000);

  for (int i = 0; i < 16; i++) {
    lastPulseByChannel[i] = -1;
  }

  // Inicializar posiciones
  for (int i = 0; i < TOTAL_SERVOS; i++) {
    currentPositions[i] = 90;
    targetPositions[i] = 90;
    setServoAngleIfChanged(servoChannels[i], 90);
  }
  setLinkedServoFromSource(90);
}

void loop() {
  if (inputComplete) {
    processSerialCommand(inputBuffer);
    inputBuffer = "";
    inputComplete = false;
  }

  if (autoMode && isMoving) {
    bool allReached = true;
    float smoothFactor = getSmoothFactor();

    for (int i = 0; i < TOTAL_SERVOS; i++) {
      float diff = targetPositions[i] - currentPositions[i];

      if (abs(diff) > THRESHOLD) {
        allReached = false;

        if (currentSpeed <= NO_SMOOTH_THRESHOLD) {
          // Movimiento seco para velocidades <= 50
          float step = (diff > 0) ? min(diff, 1.0) : max(diff, -1.0);
          currentPositions[i] += step;
        } else {
          // Suavizado ajustado según la velocidad
          float step = diff * smoothFactor;
          float maxStep = 2.0 * (currentSpeed / 100.0);
          if (abs(step) > maxStep) {
            step = (step > 0) ? maxStep : -maxStep;
          }
          currentPositions[i] += step;
        }

        // Aseguramos que no nos pasemos del objetivo
        if ((diff > 0 && currentPositions[i] > targetPositions[i]) ||
            (diff < 0 && currentPositions[i] < targetPositions[i])) {
          currentPositions[i] = targetPositions[i];
        }

        if (i == 5) { // Servo del antebrazo -> Channel 6 folgt Channel 5 (ggf. invertiert)
          setLinkedServoFromSource(currentPositions[i]);
        }

        setServoAngleIfChanged(servoChannels[i], currentPositions[i]);
      }
    }

    if (allReached) {
      isMoving = false;
      Serial.println("DONE");
    }

    delay(getMovementDelay());
  }
}

void processSerialCommand(String command) {
  command.trim();

  // Erweiterung für UART-Bridge/Test-Demo:
  // Erlaubt alternative Form: "S7 120" zusätzlich zum bestehenden "7 120"
  if (command.startsWith("S") && command.length() > 1 && isDigit(command.charAt(1))) {
    command = command.substring(1);
  }

  if (command.startsWith("MODE")) {
    autoMode = command.substring(5).toInt() == 1;
    if (autoMode) {
      receivedPositions = 0;
      isMoving = false;
    }
    return;
  }

  if (command.startsWith("SPD")) {
    int speed = command.substring(4).toInt();
    if (speed >= MIN_SPEED_VALUE && speed <= MAX_SPEED_VALUE) {
      currentSpeed = speed;
    }
    return;
  }

  int spaceIndex = command.indexOf(' ');
  if (spaceIndex != -1) {
    int servoIndex = command.substring(0, spaceIndex).toInt() - 1;
    float position = command.substring(spaceIndex + 1).toFloat();

    if (servoIndex >= 0 && servoIndex < TOTAL_SERVOS &&
        position >= MIN_SERVO_VALUE && position <= MAX_SERVO_VALUE) {

      targetPositions[servoIndex] = position;

      if (!autoMode) {
        if (abs(position - currentPositions[servoIndex]) < THRESHOLD) {
          return;
        }
        currentPositions[servoIndex] = position;

        if (servoIndex == 5) {
          setLinkedServoFromSource(position);
        }

        setServoAngleIfChanged(servoChannels[servoIndex], position);
      } else {
        receivedPositions++;
        if (receivedPositions == TOTAL_SERVOS) {
          isMoving = true;
          receivedPositions = 0;
        }
      }
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputComplete = true;
    } else {
      inputBuffer += inChar;
    }
  }
}
