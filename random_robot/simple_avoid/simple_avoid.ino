// ============================================================================
// SIMPLE AVOID — เหตุการณ์เดียวคงที่: วิ่งตรง -> เจอสิ่งกีดขวางหยุด -> เลี้ยวขวา
// เลี่ยง -> วิ่งผ่าน -> เลี้ยวซ้ายกลับแนวเดิม -> วิ่งต่อ -> จบ (เลี้ยวซ้ายครั้งเดียว
// แค่ตอนกลับแนวหลังเลี่ยง ไม่มีเลี้ยวซ้ายตั้งต้น)
// มอเตอร์/encoder คัดลอกจาก manual_control.ino/robotcurclerun.ino (ค่าจูนจริงจาก
// ฮาร์ดแวร์เดียวกัน) เซนเซอร์ตรวจสิ่งกีดขวางใช้ขาเดียวกับ obstacle_avoidance
// (HC-SR04 หน้า A0/A1) — Servo/IR รอบตัวเดินสายไว้ตามเดิมแต่ไม่ใช้ใน
// เหตุการณ์เดียวนี้ (ไม่ต้องสแกนหาทาง ทิศเลี้ยวคงที่แล้ว)
// ============================================================================
#include <ArduinoBLE.h>

// ----------------------------------------------------------------------------
// 1. HARDWARE PIN DEFINITIONS
// ----------------------------------------------------------------------------
const int ENA = 5;  // Left motor PWM
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 10; // Right motor PWM
const int IN3 = 9;
const int IN4 = 8;

const int ENCODER_L = 11;
const int ENCODER_R = 3;

const int ECHO_PIN = A0; // HC-SR04 หน้า (ตามผังขา obstacle_avoidance)
const int TRIG_PIN = A1;
// IR รอบตัว/Servo เดินสายไว้ตามฮาร์ดแวร์จริง (A2/A3/A4/A5) แต่เหตุการณ์เดียวนี้
// ทิศเลี้ยวคงที่แล้ว ไม่ต้องสแกนหาทาง จึงไม่ใช้ในโค้ดนี้

// ----------------------------------------------------------------------------
// 2. CALIBRATION (ค่าจูนจริงจาก robotcurclerun.ino/manual_control.ino)
// ----------------------------------------------------------------------------
const int DISK_SLOTS = 20;
const float WHEEL_DIAMETER_CM = 6.5;
const float TRACK_WIDTH_CM = 14.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;

int RUN_SPEED = 60000;
const int TURN_SPEED = 65535;
const int BRAKE_REVERSE_MS = 35;
const int BRAKE_HOLD_MS = 150;

const float CM_PER_SEC = 86.4;
const float DEG_PER_SEC = 310.0;
const float ENC_TRIM_L = 1.0;
const float TURN_DEG_SCALE = (360.0 / 270.0) * (360.0 / 402.5);
const float Kp_enc = 850.0;
const float Ki_enc = 20.0;
const int LAUNCH_TRIM = 14000;
const unsigned long LAUNCH_TRIM_MS = 150;

// เหตุการณ์เดียวคงที่ — ปรับสดได้ด้วยคำสั่ง (ดู parseCommand)
float TURN_DEG = 90.0;        // มุมเลี้ยวซ้าย/ขวาทั้งสองจุด
float FRONT_STOP_CM = 20.0;   // ระยะหน้าที่ถือว่าเจอสิ่งกีดขวาง
float DETOUR_CM = 40.0;       // ระยะวิ่งผ่านสิ่งกีดขวางหลังเลี้ยวขวา
float RESUME_CM = 60.0;       // ระยะวิ่งต่อหลังเลี้ยวซ้ายกลับแนวเดิม
const uint32_t ECHO_TIMEOUT_US = 26000; // A0 ไม่มี external interrupt บน UNO R4 — จำกัดรอ echo
const unsigned long PING_INTERVAL_MS = 65;
const unsigned long FRONT_INVALID_FAULT_MS = 1500; // อ่านไม่ได้ต่อเนื่องนานเท่านี้ = หยุดแจ้งเหตุ

// ----------------------------------------------------------------------------
// 3. ENCODER COUNTERS
// ----------------------------------------------------------------------------
volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;
void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }
void resetEncoders() { pulse_count_L = 0; pulse_count_R = 0; }

// ----------------------------------------------------------------------------
// 4. BLE (NUS) + STATE
// ----------------------------------------------------------------------------
const bool USE_BLE = true;
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 64);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
bool ble_connected = false;
bool was_ble_connected = false;
bool emergency_stop = false;
bool start_requested = false;
bool mission_completed = true; // ยังไม่เริ่ม จึงถือว่า "จบ" ไว้ก่อน รอ start

void sendTelemetry(const String &msg) {
  Serial.println(msg);
  if (USE_BLE && ble_connected && txChar.subscribed()) {
    txChar.writeValue(msg.c_str(), msg.length());
  }
}

// ----------------------------------------------------------------------------
// 5. ULTRASONIC (หน้า) — ตาม README obstacle_avoidance: pulseIn จำกัดรอ 26ms,
// ไม่มี echo ไม่ถือว่าทางโล่ง (คืน -1)
// ----------------------------------------------------------------------------
float readFrontCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration_us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration_us == 0) return -1.0; // timeout: อ่านไม่ได้ ไม่ถือว่าโล่ง
  return duration_us / 58.0;
}

// ----------------------------------------------------------------------------
// 6. MOTOR PRIMITIVES (คัดลอกจาก manual_control.ino)
// ----------------------------------------------------------------------------
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

void brake(int duration_ms = 150) {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 1);
  analogWrite(ENA, 65535);
  analogWrite(ENB, 65535);
  if (duration_ms > 0) delay(duration_ms);
}

void applyMotorSpeeds(int current_L, int current_R) {
  analogWrite(ENA, current_L);
  analogWrite(ENB, current_R);
}

void processBLE(); // fwd decl — เรียกกลางลูปมอเตอร์เพื่อรับ STOP ทันที

// วิ่งตรงระยะ distance_cm คงที่ (ใช้หลังเลี่ยงสิ่งกีดขวางแล้ว/วิ่งต่อจบภารกิจ)
void driveStraight(bool forward_dir, float distance_cm) {
  resetEncoders();
  digitalWrite(IN1, forward_dir ? 0 : 1);
  digitalWrite(IN2, forward_dir ? 1 : 0);
  digitalWrite(IN3, forward_dir ? 0 : 1);
  digitalWrite(IN4, forward_dir ? 1 : 0);

  int base_L = RUN_SPEED;
  int base_R = RUN_SPEED;
  int launch_bias_dir = forward_dir ? -LAUNCH_TRIM : 0;
  applyMotorSpeeds(base_L, constrain(base_R + launch_bias_dir, 23000, 65535));

  unsigned long target_duration_ms = (unsigned long)(distance_cm / CM_PER_SEC * 1000.0);
  unsigned long drive_start = millis();
  long integral_error = 0;

  while (true) {
    unsigned long elapsed = millis() - drive_start;
    if (elapsed >= target_duration_ms) break;

    long error = (long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R;
    integral_error = constrain(integral_error + error, -800, 800);
    int adjustment = constrain((int)(Kp_enc * error + Ki_enc * integral_error), -40000, 40000);

    int launch_bias = (forward_dir && elapsed < LAUNCH_TRIM_MS) ? LAUNCH_TRIM : 0;
    int current_L = constrain(base_L - adjustment, 23000, 65535);
    int current_R = constrain(base_R + adjustment - launch_bias, 23000, 65535);
    applyMotorSpeeds(current_L, current_R);

    processBLE();
    if (emergency_stop) { stop(); return; }
    delay(10);
  }

  unsigned long catchup_start = millis();
  while (!emergency_stop && labs((long)pulse_count_L - (long)pulse_count_R) > 2 && millis() - catchup_start < 300) {
    if (pulse_count_L < pulse_count_R) applyMotorSpeeds(base_L, 0);
    else applyMotorSpeeds(0, base_R);
    delay(2);
  }

  brake(100);
  stop();
  delay(50);
}

void forward(float distance_cm) { driveStraight(true, distance_cm); }

// วิ่งตรงไปข้างหน้าไม่จำกัดระยะ จนกว่าจะเจอสิ่งกีดขวางหน้า (<=FRONT_STOP_CM)
// คืน true = เจอสิ่งกีดขวางแล้วหยุด, false = อ่านเซนเซอร์ไม่ได้ต่อเนื่องนานเกินไป (fault)
bool driveUntilObstacle() {
  resetEncoders();
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  int base_L = RUN_SPEED;
  int base_R = RUN_SPEED;
  applyMotorSpeeds(base_L, constrain(base_R - LAUNCH_TRIM, 23000, 65535));

  unsigned long drive_start = millis();
  unsigned long last_ping = 0;
  unsigned long last_valid_read = millis();
  long integral_error = 0;

  while (true) {
    unsigned long now = millis();
    unsigned long elapsed = now - drive_start;

    long error = (long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R;
    integral_error = constrain(integral_error + error, -800, 800);
    int adjustment = constrain((int)(Kp_enc * error + Ki_enc * integral_error), -40000, 40000);
    int launch_bias = (elapsed < LAUNCH_TRIM_MS) ? LAUNCH_TRIM : 0;
    int current_L = constrain(base_L - adjustment, 23000, 65535);
    int current_R = constrain(base_R + adjustment - launch_bias, 23000, 65535);
    applyMotorSpeeds(current_L, current_R);

    processBLE();
    if (emergency_stop) { stop(); return false; }

    if (now - last_ping >= PING_INTERVAL_MS) {
      last_ping = now;
      float front_cm = readFrontCM();
      if (front_cm > 0) {
        last_valid_read = now;
        if (front_cm <= FRONT_STOP_CM) {
          brake(100);
          stop();
          sendTelemetry("[Avoid] เจอสิ่งกีดขวางหน้า " + String(front_cm, 1) + " ซม.");
          return true;
        }
      } else if (now - last_valid_read >= FRONT_INVALID_FAULT_MS) {
        brake(100);
        stop();
        sendTelemetry("[Avoid] FAULT: อ่านระยะหน้าไม่ได้ต่อเนื่อง");
        return false;
      }
    }

    delay(10);
  }
}

// pivot สองล้อ target_deg องศา — right_dir=true คือหมุนขวา, false คือหมุนซ้าย
void pivotTurn(bool right_dir, float target_deg) {
  resetEncoders();
  digitalWrite(IN1, right_dir ? 0 : 1);
  digitalWrite(IN2, right_dir ? 1 : 0);
  digitalWrite(IN3, right_dir ? 1 : 0);
  digitalWrite(IN4, right_dir ? 0 : 1);
  applyMotorSpeeds(TURN_SPEED, TURN_SPEED);

  float scaled_deg = target_deg * TURN_DEG_SCALE;
  float arc_cm = (scaled_deg / 360.0) * PI * TRACK_WIDTH_CM;
  unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);
  unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 3.0);
  unsigned long turn_start = millis();

  while (true) {
    unsigned long avg_pulses = (pulse_count_L + pulse_count_R) / 2;
    if (avg_pulses >= target_pulses) break;
    if (millis() - turn_start >= timeout_ms) {
      sendTelemetry("[Turn] WARNING: encoder timeout");
      break;
    }
    processBLE();
    if (emergency_stop) { stop(); return; }
    delay(2);
  }

  digitalWrite(IN1, right_dir ? 1 : 0);
  digitalWrite(IN2, right_dir ? 0 : 1);
  digitalWrite(IN3, right_dir ? 0 : 1);
  digitalWrite(IN4, right_dir ? 1 : 0);
  applyMotorSpeeds(TURN_SPEED, TURN_SPEED);
  delay(BRAKE_REVERSE_MS);
  brake(BRAKE_HOLD_MS);
  stop();
  delay(100);
}

void turn_right(float target_deg) { pivotTurn(true, target_deg); }
void turn_left(float target_deg) { pivotTurn(false, target_deg); }

// ----------------------------------------------------------------------------
// 7. เหตุการณ์เดียวคงที่
// ----------------------------------------------------------------------------
void runScenario() {
  sendTelemetry("[Scenario] วิ่งตรงจนเจอสิ่งกีดขวาง");
  if (!driveUntilObstacle()) return; // เจอ fault หรือ emergency_stop กลางทาง
  if (emergency_stop) return;

  sendTelemetry("[Scenario] เลี้ยวขวาเลี่ยง " + String(TURN_DEG, 0) + "°");
  turn_right(TURN_DEG);
  if (emergency_stop) return;

  sendTelemetry("[Scenario] วิ่งผ่านสิ่งกีดขวาง " + String(DETOUR_CM, 1) + " ซม.");
  forward(DETOUR_CM);
  if (emergency_stop) return;

  sendTelemetry("[Scenario] เลี้ยวซ้ายกลับแนวเดิม " + String(TURN_DEG, 0) + "°");
  turn_left(TURN_DEG);
  if (emergency_stop) return;

  sendTelemetry("[Scenario] วิ่งต่อ " + String(RESUME_CM, 1) + " ซม.");
  forward(RESUME_CM);
  if (emergency_stop) return;

  stop();
  mission_completed = true;
  sendTelemetry("[Scenario] จบภารกิจ");
}

// ----------------------------------------------------------------------------
// 8. COMMAND PARSER — ใช้ร่วมกันทั้ง BLE (rxChar) และ USB Serial
// ----------------------------------------------------------------------------
void reportStatus() {
  sendTelemetry("[Status] TURN_DEG=" + String(TURN_DEG, 1) + " FRONT_STOP_CM=" + String(FRONT_STOP_CM, 1) +
                " DETOUR_CM=" + String(DETOUR_CM, 1) + " RESUME_CM=" + String(RESUME_CM, 1) +
                " RUN_SPEED=" + String(RUN_SPEED) + " BLE=" + String(ble_connected ? 1 : 0) +
                " STOP=" + String(emergency_stop ? 1 : 0) + " DONE=" + String(mission_completed ? 1 : 0));
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  String lower = cmd;
  lower.toLowerCase();

  if (lower == "stop" || lower == "s") {
    emergency_stop = true;
    stop();
    sendTelemetry("[CMD] STOP");
    return;
  }
  if (lower == "start" || lower == "f") {
    emergency_stop = false;
    mission_completed = false;
    start_requested = true;
    sendTelemetry("[CMD] START");
    return;
  }
  if (lower == "v") { reportStatus(); return; }

  if (lower.startsWith("deg ")) { TURN_DEG = lower.substring(4).toFloat(); sendTelemetry("[CMD] TURN_DEG=" + String(TURN_DEG, 1)); return; }
  if (lower.startsWith("stopcm ")) { FRONT_STOP_CM = lower.substring(7).toFloat(); sendTelemetry("[CMD] FRONT_STOP_CM=" + String(FRONT_STOP_CM, 1)); return; }
  if (lower.startsWith("detour ")) { DETOUR_CM = lower.substring(7).toFloat(); sendTelemetry("[CMD] DETOUR_CM=" + String(DETOUR_CM, 1)); return; }
  if (lower.startsWith("resume ")) { RESUME_CM = lower.substring(7).toFloat(); sendTelemetry("[CMD] RESUME_CM=" + String(RESUME_CM, 1)); return; }
  if (lower.startsWith("spd ")) { RUN_SPEED = constrain(lower.substring(4).toInt(), 20000, 65535); sendTelemetry("[CMD] RUN_SPEED=" + String(RUN_SPEED)); return; }

  sendTelemetry("[CMD] ไม่รู้จัก: " + cmd);
}

void processBLE() {
  if (!USE_BLE) return;
  BLE.poll();
  BLEDevice central = BLE.central();
  ble_connected = central && central.connected();

  if (was_ble_connected && !ble_connected) {
    emergency_stop = true;
    stop();
  }
  was_ble_connected = ble_connected;

  if (ble_connected && rxChar.written()) {
    int len = rxChar.valueLength();
    const uint8_t* val = rxChar.value();
    String cmd = "";
    for (int i = 0; i < len; i++) cmd += (char)val[i];
    parseCommand(cmd);
  }
}

// ----------------------------------------------------------------------------
// 9. SETUP / LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SIMPLE AVOID READY ===");
  Serial.println("start/f = เริ่ม, stop/s = หยุด, v = สถานะ");
  Serial.println("ปรับสด: deg <n> / stopcm <n> / detour <n> / resume <n> / spd <n>");

  analogWriteResolution(16);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stop();

  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  if (USE_BLE) {
    if (BLE.begin()) {
      BLE.setLocalName("SIMPLE-AVOID");
      BLE.setDeviceName("SIMPLE-AVOID");
      BLE.setAdvertisedService(uartService);
      uartService.addCharacteristic(txChar);
      uartService.addCharacteristic(rxChar);
      BLE.addService(uartService);
      BLE.advertise();
      Serial.println("[BLE] Online! Device Name: SIMPLE-AVOID");
    } else {
      Serial.println("[BLE] Warning: BLE failed to initialize!");
    }
  }
}

void loop() {
  processBLE();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }

  if (start_requested && !emergency_stop) {
    start_requested = false;
    runScenario();
  }
}
