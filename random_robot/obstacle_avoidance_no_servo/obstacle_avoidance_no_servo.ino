// UNO R4 WiFi: wait 3 seconds, stop at 25 cm, back up, then take the farthest clear route.
// Two front IRs and servo-scanned ultrasonic. No scenarios, Bluetooth or rear IRs.
#include <Arduino.h>
#include <Servo.h>
#include "Settings.h"

constexpr int ECHO_PIN = A0, TRIG_PIN = A1, SERVO_PIN = A2;
constexpr int IR_LEFT_PIN = settings::IR_SIDES_SWAPPED ? A4 : A3;
constexpr int IR_RIGHT_PIN = settings::IR_SIDES_SWAPPED ? A3 : A4;
enum class Mode { Forward, Backup, ScanRight, ScanLeft, ChooseTurn, Turning };
Mode mode = Mode::Forward;
Servo sonarServo;
bool started = false, turnRight = true;
bool irLeft = false, irRight = false, frontValid = false;
bool rightClear = false, leftClear = false;
float rightSpaceCm = 0, leftSpaceCm = 0; // Zero: no measured range, only no-echo fallback.
float frontCm = 0;
int noEchoStreak = 0, pwmLeft = 0, pwmRight = 0;
int servoAngle = settings::SERVO_FRONT, scanSamples = 0, scanValid = 0, scanNoEcho = 0;
float scanNearest = 400;
uint32_t servoMovedAt = 0;
uint32_t bootAt = 0, phaseAt = 0, lastPing = 0, lastControl = 0;
void pauseAt(Mode next);

void setWheel(int enable, int a, int b, int value, int previous) {
  const int direction = value > 0 ? 1 : (value < 0 ? -1 : 0);
  const int oldDirection = previous > 0 ? 1 : (previous < 0 ? -1 : 0);
  if (direction != oldDirection) analogWrite(enable, 0);
  digitalWrite(a, value <= 0 ? HIGH : LOW);
  digitalWrite(b, value >= 0 ? HIGH : LOW);
  analogWrite(enable, value == 0 ? settings::BRAKE_PWM : abs(value));
}
void drive(int left, int right) {
  if (settings::LEFT_INVERTED) left = -left;
  if (settings::RIGHT_INVERTED) right = -right;
  const int channelA = constrain(settings::MOTOR_SIDES_SWAPPED ? right : left, -255, 255);
  const int channelB = constrain(int((settings::MOTOR_SIDES_SWAPPED ? left : right) *
                                     settings::RIGHT_TRIM), -255, 255);
  setWheel(settings::LEFT_PWM, settings::LEFT_A, settings::LEFT_B, channelA, pwmLeft);
  setWheel(settings::RIGHT_PWM, settings::RIGHT_A, settings::RIGHT_B, channelB, pwmRight);
  pwmLeft = channelA; pwmRight = channelB;
}
void pauseAt(Mode next) {
  drive(0, 0); // A=B=HIGH, PWM=BRAKE_PWM: brake before changing movement.
  mode = next; phaseAt = millis();
}
void clearFront() {
  frontValid = false; noEchoStreak = 0;
  lastPing = millis() - settings::PING_INTERVAL_MS;
}
void pointSonar(int angle) {
  sonarServo.write(angle); servoAngle = angle; servoMovedAt = millis();
  clearFront(); // Never use another direction's readings as front clearance.
  scanSamples = scanValid = scanNoEcho = 0; scanNearest = 400;
}
void beginScan() {
  pauseAt(Mode::Backup); // Every sweep, including retries, starts with one short retreat.
  rightClear = leftClear = false;
  rightSpaceCm = leftSpaceCm = 0;
}
bool scannedSideClear() {
  return (scanValid > settings::SONAR_SAMPLES / 2 && scanNearest >= settings::SIDE_CLEAR_CM) ||
         scanNoEcho == settings::SONAR_SAMPLES;
}
void readSensors() {
  const int detected = settings::IR_ACTIVE_LOW ? LOW : HIGH;
  irLeft = digitalRead(IR_LEFT_PIN) == detected;
  irRight = digitalRead(IR_RIGHT_PIN) == detected;
  if (millis() - servoMovedAt < settings::SERVO_SETTLE_MS ||
      millis() - lastPing < settings::PING_INTERVAL_MS) return;
  if (servoAngle != settings::SERVO_FRONT && scanSamples >= settings::SONAR_SAMPLES) return;
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const unsigned long pulse = pulseIn(ECHO_PIN, HIGH, settings::ECHO_TIMEOUT_US);
  lastPing = millis();
  const float cm = pulse * 0.0343f / 2.0f;
  const bool valid = pulse != 0 && cm >= 2 && cm <= 400;
  if (servoAngle == settings::SERVO_FRONT) {
    frontCm = cm; frontValid = valid;
    if (pulse == 0) {
      if (noEchoStreak < settings::NO_ECHO_SAMPLES) ++noEchoStreak;
    } else noEchoStreak = 0;
  } else {
    ++scanSamples;
    if (valid) { ++scanValid; if (cm < scanNearest) scanNearest = cm; }
    if (pulse == 0) ++scanNoEcho;
  }
  // Re-read after pulseIn so IR changes during its bounded wait take effect now.
  irLeft = digitalRead(IR_LEFT_PIN) == detected;
  irRight = digitalRead(IR_RIGHT_PIN) == detected;
}
bool frontReady() {
  return servoAngle == settings::SERVO_FRONT &&
         millis() - servoMovedAt >= settings::SERVO_SETTLE_MS &&
         (frontValid || noEchoStreak >= settings::NO_ECHO_SAMPLES) &&
         millis() - lastPing <= settings::RANGE_MAX_AGE_MS;
}
void updateMotion() {
  const uint32_t elapsed = millis() - phaseAt;
  switch (mode) {
    case Mode::Forward:
      if (elapsed < settings::BRAKE_MS) return;
      if (irLeft || irRight || (frontValid && frontCm <= settings::FRONT_STOP_CM)) {
        beginScan();
      } else if (frontReady()) drive(settings::RUN_PWM, settings::RUN_PWM);
      else drive(0, 0); // Resume automatically when readings recover.
      break;
    case Mode::Backup:
      if (elapsed < settings::BRAKE_MS) return;
      if (elapsed >= settings::BRAKE_MS + settings::BACKUP_MS) {
        pauseAt(Mode::ScanRight);
        pointSonar(settings::SERVO_RIGHT); // Scan only after the retreat has stopped.
      } else drive(-settings::REVERSE_PWM, -settings::REVERSE_PWM);
      break;
    case Mode::ScanRight:
      if (scanSamples < settings::SONAR_SAMPLES) return;
      rightClear = scannedSideClear();
      rightSpaceCm = scanValid > settings::SONAR_SAMPLES / 2 ? scanNearest : 0;
      pauseAt(Mode::ScanLeft); pointSonar(settings::SERVO_LEFT);
      break;
    case Mode::ScanLeft:
      if (scanSamples < settings::SONAR_SAMPLES) return;
      leftClear = scannedSideClear();
      leftSpaceCm = scanValid > settings::SONAR_SAMPLES / 2 ? scanNearest : 0;
      pauseAt(Mode::ChooseTurn); pointSonar(settings::SERVO_FRONT);
      break;
    case Mode::ChooseTurn: {
      if (elapsed < settings::BRAKE_MS) return;
      // A prolonged front read failure must not leave stale side scans queued for a turn.
      if (elapsed > settings::SERVO_SETTLE_MS + settings::RANGE_MAX_AGE_MS) {
        beginScan(); return;
      }
      if (!frontReady()) return;
      if (frontValid && frontCm <= settings::TURN_STOP_CM) {
        beginScan(); return; // Back away from the corner before the next scan.
      }
      // Compare all clear headings without a distance bias or a 25 cm measurement cap.
      // IR vetoes its side and forward. No-echo stays below any measured clear distance.
      const bool canRight = rightClear && !irRight;
      const bool canLeft = leftClear && !irLeft;
      const bool canForward = !irLeft && !irRight &&
                              (!frontValid || frontCm > settings::FRONT_STOP_CM);
      const float frontSpaceCm = frontValid ? frontCm : 0;
      if (canForward && (!canRight || frontSpaceCm >= rightSpaceCm) &&
                        (!canLeft || frontSpaceCm >= leftSpaceCm)) {
        pauseAt(Mode::Forward); // Prefer continuing straight when measured ranges tie.
        break;
      }
      if (!canRight && !canLeft) {
        beginScan(); return;
      }
      turnRight = canRight && (!canLeft || rightSpaceCm >= leftSpaceCm);
      mode = Mode::Turning; phaseAt = millis();
      // Logical wheels; drive() applies the Max_Speed channel mapping and trim.
      drive(turnRight ? settings::TURN_PWM : -settings::TURN_PWM,
            turnRight ? -settings::TURN_PWM : settings::TURN_PWM);
      break;
    }
    case Mode::Turning:
      if (elapsed >= (turnRight ? settings::TURN_RIGHT_90_MS : settings::TURN_LEFT_90_MS)) {
        pauseAt(Mode::Forward);
        clearFront(); // Measure the new heading before moving forward.
      } else if (!frontReady() || (turnRight ? irRight : irLeft) ||
                 (frontValid && frontCm <= settings::TURN_STOP_CM)) {
        beginScan(); // Brake, retreat once, then scan again after an interrupted turn.
      }
      break;
  }
}
void setup() {
  analogWriteResolution(8);
  pinMode(settings::LEFT_PWM, OUTPUT); pinMode(settings::RIGHT_PWM, OUTPUT);
  analogWrite(settings::LEFT_PWM, 0); analogWrite(settings::RIGHT_PWM, 0);
  pinMode(settings::LEFT_A, OUTPUT); pinMode(settings::LEFT_B, OUTPUT);
  pinMode(settings::RIGHT_A, OUTPUT); pinMode(settings::RIGHT_B, OUTPUT);
  drive(0, 0);
  sonarServo.attach(SERVO_PIN); // Reserve motor PWM timers before attaching Servo.
  pointSonar(settings::SERVO_FRONT);
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW); pinMode(ECHO_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT_PULLUP); pinMode(IR_RIGHT_PIN, INPUT_PULLUP);
  mode = Mode::Forward; started = false; turnRight = true;
  frontValid = false; noEchoStreak = 0; rightClear = leftClear = false;
  rightSpaceCm = leftSpaceCm = 0;
  bootAt = phaseAt = lastControl = millis();
  lastPing = bootAt - settings::PING_INTERVAL_MS;
}
void loop() {
  readSensors();
  const uint32_t now = millis();
  if (!started) {
    if (now - bootAt < settings::START_DELAY_MS) return;
    started = true;
  }
  if (now - lastControl < settings::CONTROL_MS) return;
  lastControl = now;
  updateMotion();
}
