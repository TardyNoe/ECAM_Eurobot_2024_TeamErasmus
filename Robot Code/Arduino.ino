#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_SSD1306.h>

// Constants
constexpr int TRIG_PIN = 6;
constexpr int ECHO_PIN = 7;
constexpr int RIGHT_SHAFT_PIN = 4;
constexpr int LEFT_SHAFT_PIN = 3;
constexpr int BUTTON_PIN = 2;
constexpr int TIRETTE_PIN = 8;

constexpr float FILTER_ALPHA = 0.2;
constexpr int BUFFER_SIZE = 3;
constexpr int CALIBRATION_ITERATIONS = 2000;
constexpr int LONG_PRESS_TIME = 1500;

constexpr int TIMER_TIME = 10000;

bool longPressOccurred = false;

enum Etape { CHOIX_EQUIPE,
             CHOIX_NUMERO_ROBOT };
Etape etape = CHOIX_EQUIPE;

int equipe = 0;  // 0 pour jaune, 1 pour bleu
int numeroRobot = 1;
bool isTeamSelected = false;
bool isRobotNumberSelected = false;

bool isSetupFinished = false;

unsigned long boutonPressedTime = 0;
bool isPressing = false;

unsigned long tiretteTimerStart = 0;
bool tiretteTimerRunning = false;

// Objects
Adafruit_MotorShield motorShield;
Adafruit_DCMotor* rightMotor;
Adafruit_DCMotor* leftMotor;
const int OLED_ADDRESS = 0x3C;
Adafruit_SSD1306 display(128, 64, &Wire, OLED_ADDRESS);

// Variables
float rollRate, pitchRate, yawRate, rollAngle, pitchAngle, xPosition, yPosition, currentAngle, prevCounter, deltaT, prevIterm;
float calibration[3] = {};
float acc[3] = {};
volatile int rightCounter = 0, leftCounter = 0;
int targetAngle = 90, motorPower = 0;
bool leftDirection = true, rightDirection = true, hasMoved = false;
byte serialBuffer[BUFFER_SIZE] = { 0 };
uint32_t loopTimer;

void setup() {
  Serial.begin(115200);

  Wire.setClock(100000);
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED display could not be initialized"));
    while (1)
      ;
  }
  display.display();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TIRETTE_PIN, INPUT_PULLUP);

  initializeIMU();
  initializeMotors();
}

void loop() {
  if (isSetupFinished) {
    processSerialInput();
    updateIMUReadings();
    adjustMotorControl();
    updatePositionTracking();
    transmitData();
    float distance = readUltrasonicDistance();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(equipe == 0 ? "Jaune " : "Bleu ");
    display.println(numeroRobot);
    display.setCursor(0, 20);
    if (digitalRead(TIRETTE_PIN) == HIGH) {
      if (!tiretteTimerRunning) {
        tiretteTimerStart = millis();
        tiretteTimerRunning = true;
      }

      unsigned long timerElapsed = millis() - tiretteTimerStart;
      if (timerElapsed <= TIMER_TIME) {  // 10 secondes
        display.print("Timer: ");
        display.println((TIMER_TIME - timerElapsed) / 1000);  // Affiche le temps restant en secondes
      } else {
        setMotorSpeeds(150, 150);
        display.print("X Position: ");
        display.println(xPosition);
        display.setCursor(0, 30);
        display.print("Y Position: ");
        display.println(yPosition);
        display.setCursor(0, 40);
        display.print("Current Angle: ");
        display.println(currentAngle);
        display.setCursor(0, 50);
        display.print("Distance: ");
        display.println(distance);
      }
    } else {
      display.print("Tirette presente");
      tiretteTimerRunning = false;
    }


    display.display();

  } else {
    int boutonState = digitalRead(BUTTON_PIN);

    if (boutonState == LOW) {
      if (!isPressing) {
        isPressing = true;
        longPressOccurred = false;
        boutonPressedTime = millis();
      } else if (millis() - boutonPressedTime > LONG_PRESS_TIME) {
        if (!longPressOccurred) {
          longPressOccurred = true;
          if (etape == CHOIX_EQUIPE) {
            isTeamSelected = true;
            etape = CHOIX_NUMERO_ROBOT;
          } else {
            isRobotNumberSelected = true;
            etape = CHOIX_EQUIPE;
          }
          boutonPressedTime = millis();

          if (isTeamSelected && isRobotNumberSelected) {
            isSetupFinished = true;
          }
        }
      }
    } else if (isPressing) {
      if (millis() - boutonPressedTime < LONG_PRESS_TIME) {
        if (!longPressOccurred) {
          if (etape == CHOIX_EQUIPE) {
            equipe = !equipe;
          } else {
            numeroRobot = (numeroRobot % 3) + 1;
          }
        }
      }
      isPressing = false;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    if (digitalRead(TIRETTE_PIN) == LOW) {
    display.print("Button: ");
    if (boutonState == HIGH) {
      drawCross(50, 0);
    } else {
      drawCheckmark(50, 0);
    }
    display.setCursor(0, 20);
    if (etape == CHOIX_EQUIPE) {
      display.print("Equipe: ");
      display.println(equipe == 0 ? "Jaune" : "Bleu");
    } else {
      display.print("Robot: ");
      display.println(numeroRobot);
    }
    } else {
      display.print("Tirette non detectee");
    }
    display.display();
  }
}

void drawCross(int x, int y) {
  display.drawLine(x, y, x + 10, y + 10, SSD1306_WHITE);
  display.drawLine(x + 10, y, x, y + 10, SSD1306_WHITE);
}

void drawCheckmark(int x, int y) {
  display.drawLine(x, y + 5, x + 5, y + 10, SSD1306_WHITE);
  display.drawLine(x + 5, y + 10, x + 15, y, SSD1306_WHITE);
}

void initializeIMU() {
  writeIMURegister(0x68, 0x6B, 0x00);
  writeIMURegister(0x68, 0x1A, 0x05);
  writeIMURegister(0x68, 0x1C, 0x10);
  writeIMURegister(0x68, 0x1B, 0x08);
  writeIMURegister(0x68, 0x1D, 0b00000110);

  for (int i = 0; i < CALIBRATION_ITERATIONS; ++i) {
    readIMUSignals();
    calibration[0] += rollRate;
    calibration[1] += pitchRate;
    calibration[2] += yawRate;
  }

  for (int i = 0; i < 3; ++i) {
    calibration[i] = calibration[i] / CALIBRATION_ITERATIONS;
  }
}

void readIMUSignals() {
  int16_t accLSB[3], gyro[3];

  for (int i = 0; i < 3; ++i) {
    accLSB[i] = readIMUData(0x68, 0x3B + i * 2);
    gyro[i] = readIMUData(0x68, 0x43 + i * 2);
  }

  pitchRate = gyro[0] / 65.5f * 1.55f;
  rollRate = gyro[1] / 65.5f * 1.5f;
  yawRate = -gyro[2] / 65.5f;
  for (int i = 0; i < 3; ++i) {
    acc[i] = static_cast<float>(accLSB[i]) / 4096.0f;
  }

  pitchAngle = atan(acc[1] / sqrt(acc[0] * acc[0] + acc[2] * acc[2])) * RAD_TO_DEG - 4.67f;
  rollAngle = -atan(acc[0] / sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG - 0.73f;
}

void writeIMURegister(int16_t deviceAddress, int16_t registerAddress, int16_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}

int16_t readIMUData(int16_t deviceAddress, int16_t registerAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 2);
  return Wire.read() << 8 | Wire.read();
}

void updateIMUReadings() {
  readIMUSignals();
  calibration[0] -= rollRate;
  calibration[1] -= pitchRate;
  calibration[2] -= yawRate;
}

void initializeMotors() {
  motorShield.begin();
  rightMotor = motorShield.getMotor(1);
  leftMotor = motorShield.getMotor(2);

  const int INITIAL_SPEED = 100;
  rightMotor->setSpeed(INITIAL_SPEED);
  leftMotor->setSpeed(INITIAL_SPEED);
  rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SHAFT_PIN), onRightShaftMovement, FALLING);
  attachInterrupt(digitalPinToInterrupt(LEFT_SHAFT_PIN), onLeftShaftMovement, FALLING);
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
  const int maxSpeed = 200;
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  // Set motor speed
  rightMotor->setSpeed(abs(rightSpeed));
  leftMotor->setSpeed(abs(leftSpeed));

  // Set motor direction
  leftDirection = (leftSpeed > 0) ? true : false;
  rightDirection = (rightSpeed > 0) ? true : false;

  leftMotor->run(leftDirection ? FORWARD : BACKWARD);
  rightMotor->run(rightDirection ? FORWARD : BACKWARD);

  Serial.print("test");
}

void onRightShaftMovement() {
  rightCounter += (rightDirection ? 1 : -1);
  hasMoved = true;

  unsigned long startTime = micros();
  while (micros() - startTime < 50)
    ;
}

void onLeftShaftMovement() {
  leftCounter += (leftDirection ? 1 : -1);
  hasMoved = true;

  unsigned long startTime = micros();
  while (micros() - startTime < 50)
    ;
}

void adjustMotorControl() {
  float Error = currentAngle + targetAngle;
  float Iterm = prevIterm;  //+ 0.000005*Error*dt;
  prevIterm = Iterm;
  float rotSpeed = 6 * Error + Iterm;
  setMotorSpeeds(motorPower - rotSpeed, motorPower + rotSpeed);
}

void calculateTiming() {
  int dt = micros() - loopTimer;
  currentAngle += yawRate * dt * 1e-6;
  loopTimer = micros();
}

void updatePositionTracking() {
  if (hasMoved) {
    deltaT = (rightCounter + leftCounter) / 2 - prevCounter;
    prevCounter = (rightCounter + leftCounter) / 2;
    hasMoved = false;
    xPosition += deltaT * cos(radians(currentAngle));
    yPosition += deltaT * sin(radians(currentAngle));
  }
}

void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);
  }
}

void transmitData() {
  static unsigned long lastTransmitTime = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastTransmitTime >= 10000) {
    int datax = xPosition * 10;
    int datay = yPosition * 10;
    int dataa = currentAngle * 10;
    int datadist = readUltrasonicDistance();
    if (datadist > 100) {
      datadist = 0;
    }
    byte buf[7] = {
      datax & 255, (datax >> 8) & 255,
      datay & 255, (datay >> 8) & 255,
      dataa & 255, (dataa >> 8) & 255,
      datadist & 255
    };

    float distance = readUltrasonicDistance();

    Serial.print("X Position: ");
    Serial.println(xPosition);
    Serial.print("Y Position: ");
    Serial.println(yPosition);
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
    Serial.print("Distance: ");
    Serial.println(distance);

    lastTransmitTime = currentMillis;
  }
}

float readUltrasonicDistance() {
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  while (micros() % 2 != 0)
    ;
  digitalWrite(TRIG_PIN, HIGH);
  while (micros() % 10 != 0)
    ;
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo duration in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 10000);

  // Calculate the distance in centimeters
  return duration * 0.034 / 2;  // Speed of sound is 0.034 cm/microsecond
}

float applyLowPassFilter(float currentSpeed, float previousSpeed) {
  return (FILTER_ALPHA * currentSpeed) + ((1 - FILTER_ALPHA) * previousSpeed);
}
