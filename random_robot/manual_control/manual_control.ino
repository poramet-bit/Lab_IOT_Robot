// ============================================================================
// MANUAL CONTROL — บังคับมือทีละคำสั่ง (BLE Nordic UART + USB Serial)
// แยกไฟล์จาก robotcurclerun.ino/dice_movement.ino — คัดลอกค่า pin/calibration
// ชุดเดียวกัน (ฮาร์ดแวร์ตัวเดียวกัน) เอาแค่ primitive เดินหน้า/ถอย/หมุนที่
// จบเองแล้วเบรกอัตโนมัติทุกคำสั่ง ไม่มี auto-repeat/timeout ให้จำ
// ============================================================================
#include <ArduinoBLE.h>

// ----------------------------------------------------------------------------
// 1. HARDWARE PIN DEFINITIONS (ค่าเดียวกับ robotcurclerun.ino/dice_movement.ino)
// ----------------------------------------------------------------------------
const int ENA = 5;  // Left motor PWM
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 10; // Right motor PWM
const int IN3 = 9;
const int IN4 = 8;

const int ENCODER_L = 11;
const int ENCODER_R = 3;

// ----------------------------------------------------------------------------
// 2. CALIBRATION (ค่าจูนจริงจาก robotcurclerun.ino — แก้ตรงนี้จุดเดียวถ้าฮาร์ดแวร์เปลี่ยน)
// ----------------------------------------------------------------------------
const int DISK_SLOTS = 20;
const float WHEEL_DIAMETER_CM = 6.5;
const float TRACK_WIDTH_CM = 14.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;

int RUN_SPEED = 60000;      // ปรับสดได้ด้วยคำสั่ง "spd <n>" (20000-65535)
const int TURN_SPEED = 65535; // ล็อกไว้เต็มพิกัดกันมอเตอร์กระตุกตอนหมุน (ยืนยันจาก robotcurclerun)
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

// คำสั่งเดียว = ขยับทีละสเต็ปคงที่แล้วหยุดเอง ปรับระยะ/มุมสดได้ด้วย "cm <n>"/"deg <n>"
float STEP_CM = 20.0;
float STEP_DEG = 90.0;

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

void sendTelemetry(const String &msg) {
  Serial.println(msg);
  if (USE_BLE && ble_connected && txChar.subscribed()) {
    txChar.writeValue(msg.c_str(), msg.length());
  }
}

// ----------------------------------------------------------------------------
// 5. MOTOR PRIMITIVES (คัดลอกพฤติกรรมจาก dice_movement.ino ทุกจุด)
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

void processBLE(); // fwd decl — เรียกจาก loop มอเตอร์ด้านล่างเพื่อรับ STOP กลางคัน

// วิ่งตรงระยะ distance_cm — จับเวลา (CM_PER_SEC) เป็นตัวหยุดหลัก, LM393 encoder
// (Kp_enc/Ki_enc) แก้สมดุลซ้าย/ขวาสดระหว่างวิ่ง, จบด้วย catch-up แล้วเบรก
void driveStraight(bool forward_dir, float distance_cm) {
  resetEncoders();
  digitalWrite(IN1, forward_dir ? 0 : 1);
  digitalWrite(IN2, forward_dir ? 1 : 0);
  digitalWrite(IN3, forward_dir ? 0 : 1);
  digitalWrite(IN4, forward_dir ? 1 : 0);

  int base_L = RUN_SPEED;
  int base_R = RUN_SPEED;
  int launch_bias_dir = forward_dir ? -LAUNCH_TRIM : 0; // launch trim ยืนยันจากการวิ่งหน้าเท่านั้น
  applyMotorSpeeds(base_L, constrain(base_R + launch_bias_dir, 23000, 65535));

  unsigned long target_duration_ms = (unsigned long)(distance_cm / CM_PER_SEC * 1000.0);
  unsigned long drive_start = millis();
  unsigned long last_print_time = millis();
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

    if (millis() - last_print_time >= 300) {
      sendTelemetry(" L: " + String(pulse_count_L) + " | R: " + String(pulse_count_R) +
                    " | PWM: " + String(current_L) + "/" + String(current_R));
      last_print_time = millis();
    }
    delay(10);
  }

  // Catch-up: เวลาหมดแล้วแต่ pulse ซ้าย/ขวาอาจยังไม่เท่ากัน — ขับเฉพาะล้อที่ตามหลัง
  // ต่อสั้นๆ จนกว่า pulse ซ้าย=ขวา หรือ timeout (300ms) กันเบี้ยวตกค้างก่อนเบรก
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
void backward(float distance_cm) { driveStraight(false, distance_cm); }

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
// 6. COMMAND PARSER — ใช้ร่วมกันทั้ง BLE รับ (rxChar) และ USB Serial
// ----------------------------------------------------------------------------
void reportStatus() {
  sendTelemetry("[Status] RUN_SPEED=" + String(RUN_SPEED) + " STEP_CM=" + String(STEP_CM, 1) +
                " STEP_DEG=" + String(STEP_DEG, 1) + " BLE=" + String(ble_connected ? 1 : 0) +
                " STOP=" + String(emergency_stop ? 1 : 0) +
                " ENC_L=" + String(pulse_count_L) + " ENC_R=" + String(pulse_count_R));
}

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  String lower = cmd;
  lower.toLowerCase();

  if (lower == "stop") {
    emergency_stop = true;
    stop();
    sendTelemetry("[CMD] STOP");
    return;
  }
  if (lower == "start") {
    emergency_stop = false;
    sendTelemetry("[CMD] START (ปลดล็อกฉุกเฉิน)");
    return;
  }
  if (emergency_stop) {
    sendTelemetry("[CMD] ถูกบล็อก — ส่ง START ปลดล็อกก่อน");
    return;
  }

  if (lower == "w") { forward(STEP_CM); return; }
  if (lower == "b") { backward(STEP_CM); return; }
  if (lower == "a") { turn_left(STEP_DEG); return; }
  if (lower == "d") { turn_right(STEP_DEG); return; }
  if (lower == "v") { reportStatus(); return; }

  if (lower.startsWith("cm ")) {
    STEP_CM = lower.substring(3).toFloat();
    sendTelemetry("[CMD] STEP_CM=" + String(STEP_CM, 1));
    return;
  }
  if (lower.startsWith("deg ")) {
    STEP_DEG = lower.substring(4).toFloat();
    sendTelemetry("[CMD] STEP_DEG=" + String(STEP_DEG, 1));
    return;
  }
  if (lower.startsWith("spd ")) {
    RUN_SPEED = constrain(lower.substring(4).toInt(), 20000, 65535);
    sendTelemetry("[CMD] RUN_SPEED=" + String(RUN_SPEED));
    return;
  }

  sendTelemetry("[CMD] ไม่รู้จัก: " + cmd);
}

void processBLE() {
  if (!USE_BLE) return;
  BLE.poll();
  BLEDevice central = BLE.central();
  ble_connected = central && central.connected();

  if (was_ble_connected && !ble_connected) {
    // หลุด BLE กลางคัน — เบรกทันทีและล็อกไว้ ต้องส่ง START ปลดล็อกเมื่อเชื่อมกลับ
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
// 7. SETUP / LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== MANUAL CONTROL READY ===");
  Serial.println("คำสั่ง: w=หน้า b=ถอย a=ซ้าย90 d=ขวา90 v=สถานะ stop/start");
  Serial.println("ปรับสด: cm <n> / deg <n> / spd <n>");

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

  if (USE_BLE) {
    if (BLE.begin()) {
      BLE.setLocalName("MANUAL-ROBOT");
      BLE.setDeviceName("MANUAL-ROBOT");
      BLE.setAdvertisedService(uartService);
      uartService.addCharacteristic(txChar);
      uartService.addCharacteristic(rxChar);
      BLE.addService(uartService);
      BLE.advertise();
      Serial.println("[BLE] Online! Device Name: MANUAL-ROBOT");
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
}
