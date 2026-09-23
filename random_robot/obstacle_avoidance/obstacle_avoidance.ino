// UNO R4 WiFi; open this directory as an Arduino sketch.
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Servo.h>
#include "Settings.h"
#include "Navigation.h"

constexpr int ECHO_PIN = A0, TRIG_PIN = A1, SERVO_PIN = A2;
// Four corner sensors, viewed from above with the nose pointing forward.
constexpr int IR_FRONT_LEFT_PIN = A3, IR_FRONT_RIGHT_PIN = A4;
constexpr int IR_BACK_LEFT_PIN = A5, IR_BACK_RIGHT_PIN = 2;

// Nordic UART service: write RX; subscribe to TX notifications.
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic rx("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic tx("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);
Servo sonarServo;
obstacle::Navigator navigator;
obstacle::Input input;
obstacle::Output command;
int pwmLeft = 0, pwmRight = 0;
bool bleReady = false, wasConnected = false;
uint32_t lastControl = 0;

// Scans happen while braked. No delay() for servo movement or motor sequences.
bool scanActive = false, scanDelivered = false;
int servoAngle = -1, scanIndex = 0, pingCount = 0, goodPings = 0;
int noEchoPings = 0, noEchoStreak = 0;
float nearestPing = 400;
uint32_t servoMoved = 0, lastPing = 0;
char blePrefix = 0, serialPrefix = 0;
uint32_t blePrefixAt = 0, serialPrefixAt = 0;
char report[640];
size_t reportLength = 0, serialOffset = 0, bleOffset = 0;
uint32_t lastNotify = 0;
obstacle::State reportedState = obstacle::State::Idle;

int limitPwm(int value) { return constrain(value, -255, 255); }
void setWheel(int enable, int a, int b, int value, int previous) {
  const int direction = value > 0 ? 1 : (value < 0 ? -1 : 0);
  const int oldDirection = previous > 0 ? 1 : (previous < 0 ? -1 : 0);
  if (direction != oldDirection) analogWrite(enable, 0);
  digitalWrite(a, value <= 0 ? HIGH : LOW);
  digitalWrite(b, value >= 0 ? HIGH : LOW);
  // A=B=HIGH, EN=255 is the dynamic brake used by robotline_fullspeed.
  analogWrite(enable, value == 0 ? 255 : abs(value));
}
void drive(int left, int right) {
  left = limitPwm(left);
  right = limitPwm(int(right * settings::RIGHT_TRIM));
  setWheel(settings::LEFT_PWM, settings::LEFT_A, settings::LEFT_B, left, pwmLeft);
  setWheel(settings::RIGHT_PWM, settings::RIGHT_A, settings::RIGHT_B, right, pwmRight);
  pwmLeft = left; pwmRight = right;
}
void emergencyStop(const char* reason) {
  navigator.stop(reason); command = obstacle::Output();
  drive(0, 0); // Apply immediately, without waiting for the next control tick.
  scanActive = scanDelivered = false;
  input.front.valid = input.left.valid = input.right.valid = false;
  input.front.noEcho = input.left.noEcho = input.right.noEcho = false;
  noEchoStreak = 0;
  blePrefix = serialPrefix = 0;
}
void readIR() {
  const int detected = settings::IR_ACTIVE_LOW ? LOW : HIGH;
  // A detection takes effect immediately; false positives stop conservatively.
  input.irFrontLeft = digitalRead(IR_FRONT_LEFT_PIN) == detected;
  input.irFrontRight = digitalRead(IR_FRONT_RIGHT_PIN) == detected;
  input.irBackLeft = digitalRead(IR_BACK_LEFT_PIN) == detected;
  input.irBackRight = digitalRead(IR_BACK_RIGHT_PIN) == detected;
}
void pointSonar(int angle) {
  if (servoAngle == angle) return;
  sonarServo.write(angle); servoAngle = angle; servoMoved = millis();
  noEchoStreak = 0; // Evidence from a side ray must not confirm an open front.
}
void takePing() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // A0/P014 has no external IRQ on the UNO R4 variant. Bounded pulseIn is intentional.
  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, settings::ECHO_TIMEOUT_US);
  lastPing = millis();
  const float cm = duration * 0.0343f / 2.0f;
  const bool valid = duration != 0 && cm >= 2 && cm <= 400;
  if (!scanActive) {
    if (duration == 0) { if (noEchoStreak < settings::SONAR_SAMPLES) ++noEchoStreak; }
    else noEchoStreak = 0;
    input.front.cm = cm; input.front.valid = valid; input.front.at = lastPing;
    input.front.noEcho = noEchoStreak >= settings::SONAR_SAMPLES;
    return;
  }
  ++pingCount;
  if (duration == 0) ++noEchoPings;
  if (valid) { ++goodPings; if (cm < nearestPing) nearestPing = cm; }
  if (pingCount < settings::SONAR_SAMPLES) return;
  obstacle::Range result;
  result.cm = nearestPing; result.valid = goodPings > settings::SONAR_SAMPLES / 2; result.at = lastPing;
  result.noEcho = noEchoPings == settings::SONAR_SAMPLES;
  if (scanIndex == 0) input.left = result;
  else if (scanIndex == 1) input.front = result;
  else input.right = result;
  ++scanIndex; pingCount = goodPings = noEchoPings = 0; nearestPing = 400;
  if (scanIndex == 3) {
    ++input.scanId; scanActive = false; scanDelivered = true;
    pointSonar(settings::SERVO_FRONT);
    // Keep the three scan rays until Navigator consumes this scanId.
  } else pointSonar(scanIndex == 1 ? settings::SERVO_FRONT : settings::SERVO_RIGHT);
}
void updateSonar() {
  bool wanted = navigator.scanning();
  if (wanted && scanDelivered) return;
  if (wanted && !scanActive) {
    scanActive = true; scanIndex = pingCount = goodPings = noEchoPings = noEchoStreak = 0; nearestPing = 400;
    input.front.valid = input.left.valid = input.right.valid = false;
    input.front.noEcho = input.left.noEcho = input.right.noEcho = false;
    pointSonar(settings::SERVO_LEFT);
  } else if (!wanted) {
    if (scanActive || scanDelivered || servoAngle != settings::SERVO_FRONT) {
      input.front.valid = input.front.noEcho = false; noEchoStreak = 0;
    }
    scanActive = scanDelivered = false;
    pointSonar(settings::SERVO_FRONT);
  }
  uint32_t now = millis();
  if (now - servoMoved < settings::SERVO_SETTLE_MS) return;
  if (now - lastPing >= settings::PING_INTERVAL_MS) takePing();
}
void queueStatus() {
  readIR();
  int written = snprintf(report, sizeof(report),
    "\nSTATE=%s REASON=%s SCEN=%d ROUTE=%s LEG=%d HEAD=%d\n"
    "CONTROL=TIMED LAST_ISSUE=%s CLEARANCE_RETRIES=%d\n"
    "PWM_L=%d PWM_R=%d TURN_90_MS_L/R=%lu/%lu\n"
    "IR_FL=%d IR_FR=%d IR_BL=%d IR_BR=%d FRONT_STATUS=%s FRONT_CM=%d SCAN_L_CM=%d SCAN_R_CM=%d\n"
    "EST_X_CM=%d EST_Y_CM=%d IR_ACTIVE=%s\n",
    obstacle::stateName(navigator.state), navigator.reason, navigator.selected,
    navigator.route == obstacle::Route::Top ? "TOP" : "BOTTOM", navigator.leg + 1, navigator.heading * 90,
    navigator.lastIssue, navigator.clearanceRetries,
    pwmLeft, pwmRight, (unsigned long)settings::TURN_LEFT_90_MS, (unsigned long)settings::TURN_RIGHT_90_MS,
    input.irFrontLeft, input.irFrontRight, input.irBackLeft, input.irBackRight,
    input.front.noEcho ? "NO_ECHO" : (input.front.valid ? "OK" : "UNAVAILABLE"),
    input.front.valid ? int(input.front.cm) : -1,
    input.left.valid ? int(input.left.cm) : -1, input.right.valid ? int(input.right.cm) : -1,
    int(navigator.x), int(navigator.y),
    settings::IR_ACTIVE_LOW ? "LOW" : "HIGH");
  reportLength = written < 0 ? 0 : (size_t(written) < sizeof(report) ? size_t(written) : sizeof(report) - 1);
  serialOffset = bleOffset = 0;
  reportedState = navigator.state;
}
void scenario(int scen) {
  navigator.select(scen); queueStatus();
}
void handleByte(char c, bool fromBle) {
  char& prefix = fromBle ? blePrefix : serialPrefix;
  uint32_t& prefixAt = fromBle ? blePrefixAt : serialPrefixAt;
  if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
  if (c == 's') { emergencyStop("USER_STOP"); queueStatus(); return; }
  if (millis() - prefixAt > 1000) prefix = 0;
  if (prefix == 'm' && (c == 'l' || c == 'r')) {
    prefix = 0; input.now = millis();
    navigator.testWheel(c == 'l', input); queueStatus(); return;
  }
  prefix = 0;
  if (c == 'm') { prefix = 'm'; prefixAt = millis(); return; }
  if (c >= '1' && c <= '6') { scenario(c - '0'); return; }
  input.now = millis();
  if (c == 'f') { navigator.start(input); queueStatus(); }
  else if (c == 'l' || c == 'r') { navigator.test90(c == 'l' ? -1 : 1, input); queueStatus(); }
  else if (c == 'v') queueStatus();
}
void serviceCommands() {
  if (bleReady) {
    BLE.poll();
    const bool connected = BLE.connected();
    if (wasConnected && !connected) { emergencyStop("BLE_DISCONNECTED"); queueStatus(); }
    wasConnected = connected;
    if (rx.written()) {
      uint8_t bytes[20]; int count = rx.readValue(bytes, sizeof(bytes));
      for (int i = 0; i < count; ++i) handleByte(char(bytes[i]), true);
    }
  }
  for (int i = 0; i < 20 && Serial.available(); ++i) handleByte(char(Serial.read()), false);
}
void serviceReport() {
  // Bounded writes keep motion checks ahead of diagnostics. New status replaces pending old status.
  if (Serial && serialOffset < reportLength) {
    size_t size = reportLength - serialOffset;
    if (size > 16) size = 16;
    if (Serial.availableForWrite() >= int(size)) {
      Serial.write((const uint8_t*)report + serialOffset, size); serialOffset += size;
    }
  }
  if (bleReady && BLE.connected() && tx.subscribed() && bleOffset < reportLength && millis() - lastNotify >= 20) {
    size_t size = reportLength - bleOffset;
    if (size > 20) size = 20;
    if (tx.writeValue((const uint8_t*)report + bleOffset, size)) bleOffset += size;
    lastNotify = millis();
  }
}
void setup() {
  analogWriteResolution(8);
  pinMode(settings::LEFT_PWM, OUTPUT); pinMode(settings::RIGHT_PWM, OUTPUT);
  analogWrite(settings::LEFT_PWM, 0); analogWrite(settings::RIGHT_PWM, 0);
  pinMode(settings::LEFT_A, OUTPUT); pinMode(settings::LEFT_B, OUTPUT);
  pinMode(settings::RIGHT_A, OUTPUT); pinMode(settings::RIGHT_B, OUTPUT);
  drive(0, 0);
  pinMode(TRIG_PIN, OUTPUT); digitalWrite(TRIG_PIN, LOW); pinMode(ECHO_PIN, INPUT);
  pinMode(IR_FRONT_LEFT_PIN, INPUT_PULLUP); pinMode(IR_FRONT_RIGHT_PIN, INPUT_PULLUP);
  pinMode(IR_BACK_LEFT_PIN, INPUT_PULLUP); pinMode(IR_BACK_RIGHT_PIN, INPUT_PULLUP);
  // Reserve both motor PWM timers before Servo allocates a free timer.
  sonarServo.attach(SERVO_PIN); pointSonar(settings::SERVO_FRONT);
  Serial.begin(115200);
  bleReady = BLE.begin();
  if (bleReady) {
    BLE.setLocalName("OBSTACLE-ROBOT"); BLE.setDeviceName("OBSTACLE-ROBOT");
    BLE.setAdvertisedService(uartService);
    uartService.addCharacteristic(rx); uartService.addCharacteristic(tx); BLE.addService(uartService);
    BLE.advertise();
  } else navigator.stop("BLE_INIT_FAILED_USB_AVAILABLE");
  lastControl = millis(); queueStatus();
}
void loop() {
  serviceCommands();
  readIR(); updateSonar();
  // pulseIn may take up to 26 ms; process an arriving stop before issuing motor output.
  serviceCommands();
  input.now = millis();
  if (input.now - lastControl >= settings::CONTROL_MS) {
    lastControl = input.now; readIR();
    command = navigator.update(input); drive(command.left, command.right);
    if (navigator.state != reportedState) queueStatus();
  }
  serviceReport();
}
