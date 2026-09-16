// =====================================================================
// Line Following Robot - Analog + Recovery State Machine
// TCRT5000 5CH + L298N + Arduino UNO WiFi R4
// พอร์ตมาจาก robotline_simple.ino (โปรเจกต์ robotcurclerun/IOT-Robot)
// เก็บเป็นไฟล์แยกจาก robot_line_following_PD (PD controller ตัวหลักของรถคันนี้)
//
// สลับสายมอเตอร์ทางฮาร์ดแวร์แล้ว (2026-09-15) ให้ตรงกับขาต้นฉบับของ robotline_simple.ino:
// ขา 10 = มอเตอร์ขวา, ขา 5 = มอเตอร์ซ้าย — ใช้ชื่อขา/ทิศทางแบบต้นฉบับตรงๆ ไม่ต้องแปลงอีก
// ปรับค่า calibration/tuning ได้ใน Settings.h
// =====================================================================
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include "Settings.h"

using namespace settings;

// ขา L298N ของรถคันนี้ (ตรงกับสายที่ต่อจริงหลังสลับ — เหมือน robotline_simple.ino ต้นฉบับ)
const int LEFT_PWM = 5, LEFT_A = 6, LEFT_B = 7;   // มอเตอร์ซ้าย
const int RIGHT_PWM = 10, RIGHT_A = 9, RIGHT_B = 8; // มอเตอร์ขวา

// เรียงจากซ้ายไปขวา S1..S5
const int SENSOR_PINS[5] = {A0, A1, A2, A3, A4};

static_assert(BASE_SPEED > 0 && BASE_SPEED <= MAX_SPEED && MAX_SPEED <= 255,
              "Use motor speeds in 1..MAX_SPEED, with MAX_SPEED <= 255");
static_assert(TURN_SPEED > 0 && TURN_SPEED <= MAX_SPEED &&
              SEARCH_SPEED > 0 && SEARCH_SPEED <= MAX_SPEED &&
              REVERSE_SPEED > 0 && REVERSE_SPEED <= MAX_SPEED &&
              CROSS_SPEED > 0 && CROSS_SPEED <= MAX_SPEED,
              "Turn/search/reverse/cross speeds must be in 1..MAX_SPEED");
static_assert(RIGHT_TRIM > 0 && RIGHT_TRIM <= 2 && STEERING_KP >= 0,
              "Check RIGHT_TRIM and STEERING_KP");
static_assert(SAMPLE_MS > 0 && SAMPLE_MS < CONTROL_GAP_MS,
              "SAMPLE_MS must be positive and smaller than CONTROL_GAP_MS");

int raw[5] = {}, lineMask = 0, lineGroups = 0;
float lineError = 0; // -2 = สุดซ้าย, 0 = กลาง, +2 = สุดขวา
int leftPwm = 256, rightPwm = 256; // บังคับให้รอบแรกเขียนมอเตอร์เสมอ
int lastDirection = 0;             // -1 ซ้าย, +1 ขวา, 0 ยังไม่รู้
bool stopped = false, turning = false, crossing = false;
enum class Recovery { None, BrakeBefore, Reverse, BrakeAfter, WaitLine };
Recovery recovery = Recovery::None;
const char* state = "READY";
const char* lastError = "NONE";
uint32_t lastSample = 0, lastLine = 0, turnSince = 0, blackSince = 0;
uint32_t brakeSince = 0, reverseSince = 0;

int limitPwm(int value) {
  if (value > MAX_SPEED) return MAX_SPEED;
  if (value < -MAX_SPEED) return -MAX_SPEED;
  return value;
}

// forward = a=LOW,b=HIGH ; reverse = a=HIGH,b=LOW ; value=0: a=b=HIGH (dynamic brake)
// (ตรงกับ robotline_simple.ino ต้นฉบับ — ใช้ได้ตรงๆ เพราะสลับสายมอเตอร์ให้ตรงกับขาแล้ว)
void setWheel(int enable, int a, int b, int value, int previous) {
  if (value == previous) return;
  bool sameDirection = (value > 0 && previous > 0) || (value < 0 && previous < 0);
  if (!sameDirection) {
    analogWrite(enable, 0); // ปิด bridge ก่อนสลับทิศ
    digitalWrite(a, value > 0 ? LOW : HIGH);
    digitalWrite(b, value < 0 ? LOW : HIGH);
  }
  analogWrite(enable, value == 0 ? 255 : abs(value));
}

void drive(int left, int right) {
  left = limitPwm(left);
  right = limitPwm(int(right * RIGHT_TRIM));
  setWheel(LEFT_PWM, LEFT_A, LEFT_B, left, leftPwm);
  setWheel(RIGHT_PWM, RIGHT_A, RIGHT_B, right, rightPwm);
  leftPwm = left;
  rightPwm = right;
}

void stopRobot(const char* reason) {
  if (!stopped) state = reason;
  stopped = true; // กลับมาวิ่งต่อต้อง RESET เท่านั้น
  drive(0, 0);
}

void serviceSerial() {
  static int matched = 0;
  const char* command = "STOP";
  // รับคำสั่ง STOP ได้แม้ไม่มี newline คั่น ทีละตัวอักษรข้ามรอบ loop ได้ ไม่สนตัวพิมพ์เล็ก/ใหญ่
  for (int i = 0; i < 16 && Serial.available(); ++i) {
    int ch = toupper(Serial.read());
    matched = ch == command[matched] ? matched + 1 : (ch == 'S' ? 1 : 0);
    if (matched == 4) { stopRobot("SERIAL_STOP"); matched = 0; }
  }
}

int readSensor(int pin) {
  analogRead(pin); // ทิ้งค่าแรกหลังสลับช่อง ADC
  int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  return a > b ? a : b; // median ของ 3 ค่า กันสัญญาณรบกวนจาก L298N
}

void readLine() {
  lineMask = lineGroups = 0;
  float weight = 0, weightedPosition = 0;
  bool previousBlack = false;
  for (int i = 0; i < 5; ++i) {
    raw[i] = readSensor(SENSOR_PINS[i]);
    int span = BLACK_RAW[i] - FLOOR_RAW[i];
    float darkness = span == 0 ? 0 : 1000.0f * (raw[i] - FLOOR_RAW[i]) / span;
    if (darkness < 0) darkness = 0;
    if (darkness > 1000) darkness = 1000;
    bool black = darkness >= LINE_THRESHOLD;
    if (black) {
      lineMask |= 1 << i;
      if (!previousBlack) ++lineGroups;
      weight += darkness;
      weightedPosition += (i - 2) * darkness;
    }
    previousBlack = black;
  }
  lineError = weight > 0 ? weightedPosition / weight : 0;
}

void pivot(int direction, int speed) {
  drive(direction * speed, -direction * speed);
}

void startRecovery(uint32_t now, const char* reason) {
  lastError = reason;
  recovery = Recovery::BrakeBefore;
  brakeSince = now;
  turning = crossing = false;
  state = "ERROR_BRAKE";
  drive(0, 0);
}

// คืน true ตลอดที่ recovery ยังคุมมอเตอร์อยู่ (STOP เช็คใน followLine())
bool recoverLine(uint32_t now) {
  if (recovery == Recovery::None) return false;
  if (recovery == Recovery::BrakeBefore) {
    if (now - brakeSince < BRAKE_MS) return true;
    recovery = Recovery::Reverse;
    reverseSince = now;
    state = "REVERSE";
    drive(-REVERSE_SPEED, -REVERSE_SPEED);
    return true;
  }

  // กลับมาวิ่งต่อเฉพาะตอนเจอ pattern ปกติ ไม่ใช่โค้งเดิมที่เพิ่งพลาด/เส้นแยก/แถบดำล้วน
  // ไม่ต้องใช้ encoder หรือ delay()
  bool found = lineGroups == 1 && lineMask != 31 && fabsf(lineError) < 1.5f &&
               lineMask != 7 && lineMask != 15 && lineMask != 28 && lineMask != 30;
  if (recovery == Recovery::WaitLine && !found) return true;
  if ((recovery == Recovery::Reverse || recovery == Recovery::WaitLine) && found) {
    recovery = Recovery::BrakeAfter;
    brakeSince = now;
    state = "LINE_BRAKE";
    drive(0, 0);
  }
  if (recovery == Recovery::BrakeAfter) {
    if (now - brakeSince < BRAKE_MS) return true;
    if (found) { recovery = Recovery::None; return false; }
    recovery = Recovery::Reverse; // เห็นเส้นแวบเดียวแล้วหาย: ใช้ deadline เดิมต่อ
  }
  if (now - reverseSince >= REVERSE_MS) {
    // จบรอบถอยนี้ แต่ยังอ่านเซนเซอร์ต่อ ไม่ต้อง RESET
    recovery = Recovery::WaitLine;
    state = "WAIT_LINE";
    drive(0, 0);
  } else if (recovery == Recovery::Reverse) {
    state = "REVERSE";
    drive(-REVERSE_SPEED, -REVERSE_SPEED);
  }
  return true;
}

// พฤติกรรมหลัก: แก้ branch เล็กๆ ตรงนี้เพื่อเปลี่ยนการตามเส้น
void followLine(uint32_t now) {
  if (stopped) return;
  if (recoverLine(now)) return;

  if (lineMask == 31) {
    if (!crossing) { crossing = true; blackSince = now; }
    turning = false;
    if (now - blackSince >= ALL_BLACK_MS) startRecovery(now, "ALL_BLACK");
    else { state = "CROSS"; drive(CROSS_SPEED, CROSS_SPEED); }
    return;
  }
  crossing = false;

  // กลุ่มดำแยกกันอาจเป็นเส้นข้างเคียง ห้ามเฉลี่ยข้ามกลุ่ม
  if (lineGroups > 1) { startRecovery(now, "AMBIGUOUS_LINE"); return; }
  if (lineMask == 0) {
    if (lastDirection == 0 || now - lastLine >= SEARCH_MS ||
        (turning && now - turnSince >= TURN_TIMEOUT_MS)) {
      startRecovery(now, "LINE_LOST");
    } else {
      state = "SEARCH";
      pivot(lastDirection, SEARCH_SPEED);
    }
    return;
  }
  lastLine = now;
  if (fabsf(lineError) > 0.1f) lastDirection = lineError < 0 ? -1 : 1;

  int turn = 0;
  if (lineMask == 7 || lineMask == 15 || lineError <= -1.5f) turn = -1;
  if (lineMask == 28 || lineMask == 30 || lineError >= 1.5f) turn = 1;
  if (turn != 0) {
    if (!turning) { turning = true; turnSince = now; }
    lastDirection = turn;
    if (now - turnSince >= TURN_TIMEOUT_MS) startRecovery(now, "TURN_TIMEOUT");
    else { state = "TURN"; pivot(turn, TURN_SPEED); }
    return;
  }
  turning = false;

  state = "FOLLOW";
  int correction = int(STEERING_KP * lineError);
  // เส้นอยู่ซ้าย (lineError ติดลบ): correction ติดลบ ล้อซ้ายช้าลง ล้อขวาเร็วขึ้น -> เลี้ยวเข้าเส้น
  int left = BASE_SPEED + correction, right = BASE_SPEED - correction;
  drive(left < 0 ? 0 : left, right < 0 ? 0 : right);
}

void reportStatus(uint32_t now) {
  static char report[160];
  static size_t length = 0, sent = 0;
  static uint32_t lastReport = 0;
  if (!Serial) { length = sent = 0; return; }
  if (sent == length && now - lastReport >= REPORT_MS) {
    lastReport = now;
    int n = snprintf(report, sizeof(report),
      "MS=%lu STATE=%s LAST_ERROR=%s RAW=%d,%d,%d,%d,%d MASK=%02X ERR=%d PWM=%d,%d\n",
      (unsigned long)now, state, lastError, raw[0], raw[1], raw[2], raw[3], raw[4],
      unsigned(lineMask), int(lineError * 1000), leftPwm, rightPwm);
    length = n > 0 ? (size_t(n) < sizeof(report) ? size_t(n) : sizeof(report) - 1) : 0;
    sent = 0;
  }
  int room = Serial.availableForWrite();
  size_t count = length - sent;
  if (room > 0 && count > 0) {
    if (count > size_t(room)) count = size_t(room);
    sent += Serial.write((const uint8_t*)report + sent, count);
  }
}

void updateStatusLED(uint32_t now) {
  bool on = true;
  if (stopped)
    on = now % 1500 < 100 || (now % 1500 >= 250 && now % 1500 < 350);
  else if (recovery == Recovery::WaitLine) on = now % 1000 < 150;
  else if (recovery == Recovery::Reverse) on = (now / 100) % 2 == 0;
  else if (recovery != Recovery::None) on = false; // กำลังเบรกก่อนสลับทิศ
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

void setup() {
  analogWriteResolution(8);
  pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
  analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0);
  pinMode(LEFT_A, OUTPUT); pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT); pinMode(RIGHT_B, OUTPUT);
  drive(0, 0);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  for (int i = 0; i < 5; ++i) {
    pinMode(SENSOR_PINS[i], INPUT);
    if (FLOOR_RAW[i] < 0 || FLOOR_RAW[i] > 4095 ||
        BLACK_RAW[i] < 0 || BLACK_RAW[i] > 4095 || FLOOR_RAW[i] == BLACK_RAW[i])
      stopRobot("INVALID_SENSOR_CONFIG");
  }
  if (LINE_THRESHOLD < 1 || LINE_THRESHOLD > 1000) stopRobot("INVALID_THRESHOLD");
  lastSample = lastLine = millis();
}

void loop() {
  serviceSerial(); // STOP ทำงานได้แม้ระหว่างรอบอ่านเซนเซอร์
  uint32_t now = millis();
  if (now - lastSample >= SAMPLE_MS) {
    readLine();
    now = millis();
    if ((leftPwm != 0 || rightPwm != 0) && now - lastSample > CONTROL_GAP_MS)
      stopRobot("CONTROL_OVERRUN");
    lastSample = now;
    followLine(now);
    updateStatusLED(now);
  }
  reportStatus(millis());
}
