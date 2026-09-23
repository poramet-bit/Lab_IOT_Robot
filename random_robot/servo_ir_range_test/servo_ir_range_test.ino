// Bench test for obstacle_avoidance_auto: measures the trigger distance of each IR
// sensor and confirms which physical side the servo angles point at.
//
// Motors are never driven here, so the robot can sit still on the bench.
//
// How to use:
//   1. Upload, open Serial Monitor at 9600 baud.
//   2. The servo parks facing front. Slide a flat obstacle (a book works) straight
//      toward a front IR sensor, slowly, keeping it in the ultrasonic beam.
//   3. When the IR flips to BLOCKED the sketch prints the sonar distance at that
//      instant: that is the IR trigger distance. Pull the obstacle back to get the
//      release distance too (IR modules have hysteresis, the two differ).
//   4. Repeat a few times; use the average to pick FRONT_STOP_CM in the main sketch.
//      Rule of thumb: FRONT_STOP_CM slightly above the IR trigger distance, so the
//      sonar reacts first and the IR only acts as a backup.
//   5. Type L, F or R in Serial Monitor to point the servo left/front/right and read
//      which side it actually faces. If L faces right, set SERVO_MIRRORED = true in
//      obstacle_avoidance_auto.ino.
//
// Back sensors are printed as raw state only: the ultrasonic looks forward, so it
// cannot measure their range. Measure those with a ruler while watching the state.

#include <Servo.h>

constexpr int TRIG_PIN = A1, ECHO_PIN = A0, SERVO_PIN = A2;
constexpr int SERVO_LEFT = 180, SERVO_FRONT = 90, SERVO_RIGHT = 0;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

constexpr int IR_FRONT_LEFT = A3, IR_FRONT_RIGHT = A4;
constexpr int IR_BACK_LEFT = A5, IR_BACK_RIGHT = 2;
constexpr bool IR_ACTIVE_LOW = true;

constexpr int SENSOR_COUNT = 4;
const int IR_PINS[SENSOR_COUNT] = {IR_FRONT_LEFT, IR_FRONT_RIGHT, IR_BACK_LEFT, IR_BACK_RIGHT};
const char *IR_NAMES[SENSOR_COUNT] = {"front-left", "front-right", "back-left", "back-right"};
const bool IR_FACES_SONAR[SENSOR_COUNT] = {true, true, false, false};

bool lastBlocked[SENSOR_COUNT];
unsigned long lastReport = 0;

Servo sonar;

bool irBlocked(int pin) {
  return digitalRead(pin) == (IR_ACTIVE_LOW ? LOW : HIGH);
}

// Median of three pings: a single reading jitters by a few cm and would make the
// recorded trigger distance noisier than the thing being measured.
float pingCm() {
  float samples[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    samples[i] = duration == 0 ? 400.0f : duration * 0.0343f / 2.0f;
    delay(10);
  }
  if (samples[0] > samples[1]) { float t = samples[0]; samples[0] = samples[1]; samples[1] = t; }
  if (samples[1] > samples[2]) { float t = samples[1]; samples[1] = samples[2]; samples[2] = t; }
  if (samples[0] > samples[1]) { float t = samples[0]; samples[0] = samples[1]; samples[1] = t; }
  return samples[1];
}

void handleServoCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'L' || c == 'l') { sonar.write(SERVO_LEFT); Serial.println("servo -> SERVO_LEFT (180)"); }
    else if (c == 'R' || c == 'r') { sonar.write(SERVO_RIGHT); Serial.println("servo -> SERVO_RIGHT (0)"); }
    else if (c == 'F' || c == 'f') { sonar.write(SERVO_FRONT); Serial.println("servo -> SERVO_FRONT (90)"); }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  for (int i = 0; i < SENSOR_COUNT; i++) pinMode(IR_PINS[i], INPUT_PULLUP);

  sonar.attach(SERVO_PIN);
  sonar.write(SERVO_FRONT);
  delay(500);

  for (int i = 0; i < SENSOR_COUNT; i++) lastBlocked[i] = irBlocked(IR_PINS[i]);

  Serial.println("IR range test. Keys: L/F/R move servo. Motors stay off.");
  Serial.println("Move an obstacle slowly toward a front IR and read the cm printed on the flip.");
}

void loop() {
  handleServoCommand();

  float cm = pingCm();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    bool blocked = irBlocked(IR_PINS[i]);
    if (blocked == lastBlocked[i]) continue;
    lastBlocked[i] = blocked;

    Serial.print(blocked ? "TRIGGER  " : "RELEASE  ");
    Serial.print(IR_NAMES[i]);
    if (IR_FACES_SONAR[i]) {
      Serial.print("  at sonar ");
      Serial.print(cm, 1);
      Serial.println(" cm");
    } else {
      Serial.println("  (rear sensor: measure with a ruler, sonar faces forward)");
    }
  }

  // Steady background line so the live distance is visible while positioning.
  if (millis() - lastReport >= 500) {
    lastReport = millis();
    Serial.print("sonar="); Serial.print(cm, 1); Serial.print("cm  ");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(IR_NAMES[i]); Serial.print("=");
      Serial.print(lastBlocked[i] ? "BLOCKED" : "clear");
      Serial.print(i == SENSOR_COUNT - 1 ? "\n" : "  ");
    }
  }

  delay(20);
}
