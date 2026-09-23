#include <Servo.h>

// Motor wiring/direction from IOT-Robot/Robot/robotline_fullspeed.
constexpr int LEFT_PWM = 5, LEFT_A = 6, LEFT_B = 7;
constexpr int RIGHT_PWM = 10, RIGHT_A = 9, RIGHT_B = 8;
constexpr float RIGHT_TRIM = 46800.0f / 55500.0f;

constexpr int TRIG_PIN = A1, ECHO_PIN = A0, SERVO_PIN = A2;
constexpr int SERVO_LEFT = 180, SERVO_FRONT = 90, SERVO_RIGHT = 0;
constexpr uint32_t SERVO_SETTLE_MS = 300;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

// Front pair sees obstacles ahead; back pair only checked before reversing.
constexpr int IR_FRONT_LEFT = A3, IR_FRONT_RIGHT = A4;
constexpr int IR_BACK_LEFT = A5, IR_BACK_RIGHT = 2;
constexpr bool IR_ACTIVE_LOW = true; // verify with a Serial print; invert if opposite.

// Tune these on the real robot.
constexpr int RUN_PWM = 180, TURN_PWM = 180, REVERSE_PWM = 150;
constexpr float FRONT_STOP_CM = 20.0f, SIDE_MIN_CM = 25.0f;
constexpr unsigned long TURN_90_MS = 400, BACKUP_MS = 400;

Servo sonar;

void setWheel(int pwmPin, int aPin, int bPin, int value) {
  digitalWrite(aPin, value <= 0 ? HIGH : LOW);
  digitalWrite(bPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, value == 0 ? 255 : abs(value)); // A=B=HIGH, EN=255 brakes.
}

void drive(int left, int right) {
  setWheel(LEFT_PWM, LEFT_A, LEFT_B, left);
  setWheel(RIGHT_PWM, RIGHT_A, RIGHT_B, int(right * RIGHT_TRIM));
}

void stopMotors() { drive(0, 0); }

bool irTriggered(int pin) {
  return digitalRead(pin) == (IR_ACTIVE_LOW ? LOW : HIGH);
}

float pingCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  return duration == 0 ? 400.0f : duration * 0.0343f / 2.0f;
}

float lookAt(int angle) {
  sonar.write(angle);
  delay(SERVO_SETTLE_MS);
  return pingCm();
}

void turnLeft(unsigned long ms) { drive(-TURN_PWM, TURN_PWM); delay(ms); stopMotors(); }
void turnRight(unsigned long ms) { drive(TURN_PWM, -TURN_PWM); delay(ms); stopMotors(); }

void avoidObstacle() {
  stopMotors();

  float left = lookAt(SERVO_LEFT);
  float right = lookAt(SERVO_RIGHT);
  sonar.write(SERVO_FRONT);
  delay(SERVO_SETTLE_MS);

  bool leftBlocked = left < SIDE_MIN_CM;
  bool rightBlocked = right < SIDE_MIN_CM;

  if (leftBlocked && rightBlocked) {
    if (!irTriggered(IR_BACK_LEFT) && !irTriggered(IR_BACK_RIGHT)) {
      drive(-REVERSE_PWM, -REVERSE_PWM);
      unsigned long start = millis();
      while (millis() - start < BACKUP_MS &&
             !irTriggered(IR_BACK_LEFT) && !irTriggered(IR_BACK_RIGHT)) {}
      stopMotors();
    }
    turnRight(TURN_90_MS); // boxed in on both sides: spin and let the next loop re-scan.
    return;
  }

  if (right >= left) turnRight(TURN_90_MS);
  else turnLeft(TURN_90_MS);
}

void setup() {
  Serial.begin(9600);

  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_A, OUTPUT); pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_A, OUTPUT); pinMode(RIGHT_B, OUTPUT);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IR_FRONT_LEFT, INPUT_PULLUP);
  pinMode(IR_FRONT_RIGHT, INPUT_PULLUP);
  pinMode(IR_BACK_LEFT, INPUT_PULLUP);
  pinMode(IR_BACK_RIGHT, INPUT_PULLUP);

  sonar.attach(SERVO_PIN);
  sonar.write(SERVO_FRONT);
  delay(500);

  Serial.println("autonomous obstacle avoidance start");
  delay(1500); // time to place the robot down before it starts moving
}

void loop() {
  bool frontIrBlocked = irTriggered(IR_FRONT_LEFT) || irTriggered(IR_FRONT_RIGHT);
  float frontCm = pingCm();

  if (frontIrBlocked || frontCm < FRONT_STOP_CM) avoidObstacle();
  else drive(RUN_PWM, RUN_PWM);
}
