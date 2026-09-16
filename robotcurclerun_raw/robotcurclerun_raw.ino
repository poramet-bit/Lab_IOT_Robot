#include <Wire.h>
#include "Arduino_LED_Matrix.h"
#include "/home/poramet/Documents/Lab_IOT_Robot/animation/animation.h"
#include <ArduinoBLE.h>

// Timed route: RUN_SPEED/LEFT_RATIO and RIGHT_RUN_SPEED tune forward PWM.
// L_SPIN_MS/R_SPIN_MS tune turns. LEG1..4_FORWARD_CM tune distances.
// L/R denote code channels; keep the existing wiring assignment.
// One combined forward + spin summary is emitted at the end of each leg.
// ============================================================================
// 1. HARDWARE PIN DEFINITIONS
// ============================================================================
// Left Motor (L298N)
// hardware test ยืนยันแล้วว่าคอมเมนต์เดิม (ENA=ขวา, ENB=ซ้าย) สลับข้างจริง —
// มองจากด้านบนตัวหุ่น ขา 10/9/8 ต่ออยู่กับมอเตอร์ฝั่งขวา ขา 5/7/6 ต่อฝั่งซ้าย
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;

// Right Motor (L298N)
const int ENB = 10;
const int IN3 = 9;
const int IN4 = 8;

// LM393 Speed Encoders
// คู่จับคู่จริงทางฮาร์ดแวร์ (ยืนยันจากการ converge ของ loop): encoder ขา11
// อยู่บนล้อเดียวกับมอเตอร์ขา10, encoder ขา3 อยู่บนล้อเดียวกับมอเตอร์ขา5
// ตอนนี้ ENA=5 (ซ้าย), ENB=10 (ขวา) แล้ว จึงต้อง encoder ขา3=ซ้าย, ขา11=ขวา
// ให้ตรงกับมอเตอร์คนละตัว (ไม่งั้น pulse ที่วัดกับ PWM ที่สั่งจะคนละล้อกัน)
const int ENCODER_L = 3;
const int ENCODER_R = 11;

// Objects
ArduinoLEDMatrix matrix;

// ============================================================================
// 2. ROBOT PHYSICAL GEOMETRY & CALIBRATION
// ============================================================================

// ============================================================================
// 16-BIT SPEED & BALANCE CONFIGURATION (analogWriteResolution = 16-bit: 0 - 65535)
// ============================================================================
int RUN_SPEED = 65500; // Base PWM copied from test_drive_ble.
const float LEFT_RATIO = 41000.0f / 55000.0f; // Forward L PWM = 48827 at RUN_SPEED 65500.
const int RIGHT_RUN_SPEED = 48000; // Tune R independently, preserving L PWM.
const unsigned long L_SPIN_MS = 2860; // Mean of BLE l durations: 1675, 1676, 1753 ms.
const unsigned long R_SPIN_MS = 4053; // 4900ms measured at TURN_SPEED=45000; rescaled to 65000 (4900*45000/65000).
int TURN_SPEED = 65000;  // ความเร็วตอนหมุนเลี้ยว 16-bit สูงสุดเต็มพิกัด 100% (ป้องกันมอเตอร์กระตุกเวลาหมุนบนพื้น)
int BRAKE_REVERSE_MS = 35; // สวนกระแสมอเตอร์เพื่อหยุดแรงเฉื่อยสะบัดทันที (ms)
int BRAKE_HOLD_MS = 150;   // ล็อกล้อด้วยระบบ Dynamic Brake (ms)
const int FORWARD_BRAKE_REVERSE_MS = 20; // Short counter-torque before the forward stop.

// ตัวกำหนดระยะทาง/มุม: เปลี่ยนจากนับ pulse encoder เป็นจับเวลา (delay/millis)
// แทน — วิ่ง/หมุนตามเวลาที่คำนวณไว้ตรงๆ ไม่รอ pulse ถึงเป้าหมายอีกต่อไป
// ประมาณค่าจาก log ฮาร์ดแวร์จริงที่ผ่านมา (ไม่ใช่ทฤษฎีล้วน) แต่ยังหยาบ —
// ความเร็วจริงเปลี่ยนตามแบตเตอรี่/พื้นผิว ต้องคอยเทียบกับระยะจริงแล้วปรับสองค่านี้
float CM_PER_SEC  = 86.4;  // ปรับหลังเปลี่ยนมอเตอร์ Hyper Dash 3: สั่ง 150cm ได้ระยะจริง 240cm
                            // (เร็วกว่าค่าประเมินเดิม 54.0 อยู่ 1.6 เท่า -> 54.0*1.6=86.4)

// Motor Balance Tuning 16-bit
// Legs 1-4: L = RUN_SPEED * LEFT_RATIO, R = RIGHT_RUN_SPEED.

// ============================================================================
// ROUTE PARAMETERS — ค่าแยกอิสระต่อจุด ปรับจุดไหนไม่กระทบจุดอื่น
// ============================================================================
// SPEED_L/SPEED_R ต่อ leg: ความเร็ว PWM (16-bit, 0-65535) ของล้อซ้าย/ขวา
// เฉพาะช่วงวิ่งตรงของ leg นั้น
int powersaver(int basespeed, float multiple);

const float LEG1_FORWARD_CM = 640;  //170
const int   LEG1_SPEED_L    = powersaver(RUN_SPEED, LEFT_RATIO);
const int   LEG1_SPEED_R    = RIGHT_RUN_SPEED;

const float LEG2_FORWARD_CM = 890; //205
const int   LEG2_SPEED_L    = powersaver(RUN_SPEED, LEFT_RATIO);
const int   LEG2_SPEED_R    = RIGHT_RUN_SPEED;

const float LEG3_FORWARD_CM = 640;
const int   LEG3_SPEED_L    = powersaver(RUN_SPEED, LEFT_RATIO);
const int   LEG3_SPEED_R    = RIGHT_RUN_SPEED;

const float LEG4_FORWARD_CM = 890;
const int   LEG4_SPEED_L    = powersaver(RUN_SPEED, LEFT_RATIO);
const int   LEG4_SPEED_R    = RIGHT_RUN_SPEED;

const float LEG5_FORWARD_CM = 0;
const int   LEG5_SPEED_L    = 0;
const int   LEG5_SPEED_R    = 0;

// ============================================================================
// 3. ENCODER COUNTERS & ISRs
// ============================================================================
volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }

void resetEncoders() {
  pulse_count_L = 0;
  pulse_count_R = 0;
}


// ============================================================================
// 4. BLUETOOTH LOW ENERGY (BLE) - NORDIC UART SERVICE (iOS & Android)
// ============================================================================
const bool USE_BLE = true; // เปิดใช้งาน Bluetooth ไร้สายสำหรับ iPhone
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
// buffer 64 ไบต์เดิมเล็กไป — ข้อความยาว (เช่น log ตอน turn ที่มี Rev L/R + Yaw) โดนตัดกลางคัน
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 200);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
bool ble_init_ok = false;
bool ble_connected = false;
bool emergency_stop = false;

void sendTelemetry(const String &msg) {
  if (Serial) Serial.println(msg);
  if (USE_BLE && ble_connected && txChar.subscribed()) {
    txChar.writeValue(msg.c_str(), msg.length());
  }
}

void processBLE() {
  if (!USE_BLE) return;
  BLE.poll();
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    ble_connected = true;
    if (rxChar.written()) {
      int len = rxChar.valueLength();
      const uint8_t* val = rxChar.value();
      String cmd = "";
      for (int i = 0; i < len; i++) cmd += (char)val[i];
      cmd.trim();
      cmd.toUpperCase();
      if (cmd == "STOP") {
        emergency_stop = true;
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        sendTelemetry("[BLE CMD] EMERGENCY STOP ACTIVATED!");
      } else if (cmd == "START") {
        emergency_stop = false;
        sendTelemetry("[BLE CMD] START RECEIVED!");
      }
    }
  } else {
    ble_connected = false;
  }
}

// ============================================================================
// 4.1 MPU-6050 GYROSCOPE DRIVER (Auto-Detect & Fallback) — รับค่ามาอ่านเฉยๆ
// ไม่มี Kp_gyro / correction ใดๆ ใช้แค่ log current_yaw ไว้เทียบกับมุมที่สั่ง
// ============================================================================
const bool USE_MPU6050 = true; // ต่อสาย A4(SDA)/A5(SCL) แล้ว

const int MPU_ADDR = 0x68;
bool has_mpu = false;
float gyro_z_offset = 0;
float current_yaw = 0;
bool gyro_valid = false;
bool gyro_saturated = false;
unsigned long last_drive_ms = 0;
const int DISK_SLOTS = 20;
const float CM_PER_PULSE = 3.14159265f * 6.5f / DISK_SLOTS;
const float TRACK_WIDTH_CM = 14.5f;
unsigned long last_gyro_time = 0;

bool initMPU6050() {
  if (!USE_MPU6050) {
    Serial.println("[IMU] MPU-6050 disabled (USE_MPU6050 = false).");
    has_mpu = false;
    return false;
  }

  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  Wire.setWireTimeout(10000, true);
  #endif

  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("[IMU] MPU-6050 not detected.");
    has_mpu = false;
    return false;
  }

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set Gyro full scale range to +/- 250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Serial.println("[IMU] MPU-6050 detected! Calibrating gyro Z (keep robot still)...");
  long sum_z = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    if (Wire.available() >= 2) {
      int16_t raw_z = (Wire.read() << 8) | Wire.read();
      sum_z += raw_z;
    }
    delay(3);
  }
  gyro_z_offset = (float)sum_z / 200.0;
  current_yaw = 0;
  last_gyro_time = micros();
  has_mpu = true;
  Serial.println("[IMU] Calibration complete! Gyro reading (log-only, no correction) active.");
  return true;
}

void updateYaw() {
  if (!has_mpu) return;

  unsigned long now = micros();
  float dt = (now - last_gyro_time) / 1000000.0;
  last_gyro_time = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  if (Wire.available() >= 2) {
    int16_t raw_z = (Wire.read() << 8) | Wire.read();
    if (raw_z >= 32700 || raw_z <= -32700) gyro_saturated = true;
    float rate_z = (raw_z - gyro_z_offset) / 131.0; // 131 LSB/(deg/s)
    if (abs(rate_z) > 0.15) { // Filter sensor noise
      current_yaw += rate_z * dt;
    }
  } else {
    gyro_valid = false;
  }
}

void resetYaw() {
  gyro_valid = has_mpu;
  gyro_saturated = false;
  current_yaw = 0;
  last_gyro_time = micros();
}

// ============================================================================
// 5. MOTOR CONTROL PRIMITIVES
// ============================================================================
// Scale raw PWM; use a multiplier from 0.0f to 1.0f to reduce power.
// Fractional results are truncated. No clamping, matching this raw sketch.
int powersaver(int basespeed, float multiple) {
  basespeed = static_cast<int>(basespeed * multiple);
  return basespeed; 
}

// Set the L298N direction inputs in left-to-right pin order.
void setMotorDirections(int leftIn1, int leftIn2, int rightIn3, int rightIn4) {
  digitalWrite(IN1, leftIn1);
  digitalWrite(IN2, leftIn2);
  digitalWrite(IN3, rightIn3);
  digitalWrite(IN4, rightIn4);
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  setMotorDirections(0, 0, 0, 0);
}

// Active Electronic Brake (L298N Dynamic Brake: short-circuits coils to lock wheels rigid)
void brake(int duration_ms = 150) {
  setMotorDirections(1, 1, 1, 1);
  analogWrite(ENA, 65535);
  analogWrite(ENB, 65535);
  if (duration_ms > 0) {
    delay(duration_ms);
  }
}

// Simultaneous Motor Power Application
// ข้อเท็จจริงของฮาร์ดแวร์จริง (ยืนยันแล้ว): ENA (ขา 5) คือมอเตอร์ซ้าย, ENB (ขา 10) คือมอเตอร์ขวา
void applyMotorSpeeds(int current_L, int current_R) {
  analogWrite(ENA, current_L); // ส่งไฟ current_L เข้ามอเตอร์ซ้ายจริง (ENA ขา 5)
  analogWrite(ENB, current_R); // ส่งไฟ current_R เข้ามอเตอร์ขวาจริง (ENB ขา 10)
}

// ============================================================================
// 6. LED MATRIX DISPLAY TRANSFORM HELPERS
// ============================================================================
void loadTransformedFrame(const uint32_t frame[4], bool flipX, bool flipY) {
  uint32_t transformed[4] = {0, 0, 0, frame[3]};
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 12; x++) {
      int srcBit = y * 12 + x;
      int srcWord = srcBit / 32;
      int srcPos = 31 - (srcBit % 32);
      bool isSet = (frame[srcWord] >> srcPos) & 1;
      if (isSet) {
        int new_x = flipX ? (11 - x) : x;
        int new_y = flipY ? (7 - y) : y;
        int dstBit = new_y * 12 + new_x;
        int dstWord = dstBit / 32;
        int dstPos = 31 - (dstBit % 32);
        transformed[dstWord] |= (1UL << dstPos);
      }
    }
  }
  matrix.loadFrame(transformed);
}


void loadFlippedYFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, false, true);
}


// ============================================================================
// 7. HIGH-SPEED NAVIGATION APIs (RAW — ไม่มี heading-lock correction/clamp)
// ============================================================================

// Forward by distance — จ่าย PWM raw ตรงๆ ตลอดระยะทาง ไม่มีการปรับแก้ใดๆ
void forward(int speed_motorL, int speed_motorR, int B_L = 0, int B_R = 0, float distance_cm = 0) {
  resetEncoders();
  resetYaw();

  // Set motor directions forward
  setMotorDirections(0, 1, 0, 1);

  int base_L = (speed_motorL - B_R);
  int base_R = (speed_motorR - B_L);

  applyMotorSpeeds(base_L, base_R);

  // If distance is 0 or not specified, run one single animation cycle (default mode)
  if (distance_cm <= 0) {
    int total_frames = sizeof(walk) / sizeof(walk[0]);
    for (int i = 0; i < total_frames; i++) {
      loadFlippedYFrame(walk[i]);
      delay(walk[i][3]);
    }
    brake(BRAKE_HOLD_MS);
    return;
  }

  // Running for specified distance (cm) — จับเวลาแทนนับ pulse
  unsigned long target_duration_ms = (unsigned long)(distance_cm / CM_PER_SEC * 1000.0);
  unsigned long drive_start = millis();
  int total_frames = sizeof(walk) / sizeof(walk[0]);
  int anim_frame = 0;
  unsigned long last_anim_time = millis();

  while (true) {
    if (millis() - drive_start >= target_duration_ms) break;

    updateYaw();

    // LED Matrix Animation
    if (millis() - last_anim_time >= walk[anim_frame][3]) {
      loadFlippedYFrame(walk[anim_frame]);
      anim_frame = (anim_frame + 1) % total_frames;
      last_anim_time = millis();
    }

    processBLE();
    if (emergency_stop) {
      stop();
      break;
    }


    delay(10);
  }

  last_drive_ms = millis() - drive_start;

  // Cancel forward momentum before holding both wheels with dynamic braking.
  if (!emergency_stop && FORWARD_BRAKE_REVERSE_MS > 0) {
    applyMotorSpeeds(0, 0);
    setMotorDirections(1, 0, 1, 0);
    applyMotorSpeeds(base_L, base_R);
    delay(FORWARD_BRAKE_REVERSE_MS);
  }
  brake(BRAKE_HOLD_MS);
  delay(50); // Settle with the brake still engaged.
  updateYaw();
}

// Single-wheel pivot: right wheel stays stationary, left wheel drives forward.
void pivotLeftForward(unsigned long duration_ms, int speed = -1) {
  if (speed <= 0) speed = TURN_SPEED;
  resetEncoders();
  resetYaw();

  // Code L drives forward; code R stays dynamically braked.
  setMotorDirections(0, 1, 1, 1);

  applyMotorSpeeds(speed, 65535);

  unsigned long target_duration_ms = duration_ms;
  unsigned long pivot_start = millis();
  while (millis() - pivot_start < target_duration_ms) {
    updateYaw();
    delay(2);
  }

  last_drive_ms = millis() - pivot_start;

  // Counter-torque on the moving wheel while the other remains braked.
  analogWrite(ENA, 0);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  applyMotorSpeeds(speed, 65535);
  delay(BRAKE_REVERSE_MS);

  // Brake both wheels and keep braking through the settle pause.
  brake(BRAKE_HOLD_MS);
  updateYaw();
  delay(100);
}

// Single-wheel pivot: left wheel stays stationary, right wheel drives forward.
void pivotRightForward(unsigned long duration_ms, int speed = -1) {
  if (speed <= 0) speed = TURN_SPEED;
  resetEncoders();
  resetYaw();

  // Code R drives forward; code L stays dynamically braked.
  setMotorDirections(1, 1, 0, 1);

  applyMotorSpeeds(65535, speed);

  unsigned long target_duration_ms = duration_ms;
  unsigned long pivot_start = millis();
  while (millis() - pivot_start < target_duration_ms) {
    updateYaw();
    delay(2);
  }

  last_drive_ms = millis() - pivot_start;

  // Counter-torque on the moving wheel while the other remains braked.
  analogWrite(ENB, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  applyMotorSpeeds(65535, speed);
  delay(BRAKE_REVERSE_MS);

  // Brake both wheels and keep braking through the settle pause.
  brake(BRAKE_HOLD_MS);
  updateYaw();
  delay(100);
}

// ============================================================================
// 8. SETUP
// ============================================================================
void setup() {
  // BLE ก่อนอย่างอื่นทั้งหมด — ให้มือถือจับสัญญาณได้ทันทีที่ไฟเข้าบอร์ด
  // ไม่ต้องรอ Serial enumeration / matrix / motor / gyro calibration
  if (USE_BLE) {
    ble_init_ok = BLE.begin();
    if (ble_init_ok) {
      BLE.setLocalName("IOT-ROBOT");
      BLE.setDeviceName("IOT-ROBOT");
      BLE.setAdvertisedService(uartService);
      uartService.addCharacteristic(txChar);
      uartService.addCharacteristic(rxChar);
      BLE.addService(uartService);
      BLE.advertise();
    }
  }

  Serial.begin(9600);
  delay(1000); // 1-second delay for USB Serial enumeration & connection

  Serial.println();
  Serial.println("=========================================");
  Serial.println("   IOT ROBOT SYSTEM ONLINE - RAW MODE     ");
  Serial.println("=========================================");

  matrix.begin();

  // Motor pins & 16-bit PWM Resolution (0 - 65535)
  analogWriteResolution(16);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stop();

  // Encoders (diagnostic only in raw mode — no correction reads them)
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);

  // Gyro (diagnostic only in raw mode — read/log yaw, no correction)
  initMPU6050();

  // สถานะ BLE (เริ่มไปแล้วตั้งแต่บรรทัดบนสุดของ setup) — พิมพ์ log ตอนนี้เพราะ Serial เพิ่งพร้อม
  if (USE_BLE) {
    if (ble_init_ok) {
      Serial.println("[BLE] Bluetooth Online! Device Name: IOT-ROBOT");
    } else {
      Serial.println("[BLE] Warning: BLE failed to initialize!");
    }
  }

  Serial.println("System Ready! Waiting to start...");
}

// ============================================================================
// 9. LOOP - SEQUENCE RUNNER
// ============================================================================
unsigned long forward_count_L = 0;
unsigned long forward_count_R = 0;
float forward_yaw = 0;
unsigned long forward_ms = 0;
bool forward_gyro_valid = false;
bool forward_gyro_saturated = false;

void captureForward() {
  noInterrupts();
  forward_count_L = pulse_count_L;
  forward_count_R = pulse_count_R;
  interrupts();
  forward_yaw = current_yaw;
  forward_ms = last_drive_ms;
  forward_gyro_valid = gyro_valid;
  forward_gyro_saturated = gyro_saturated;
}

// Send the completed report in 20-byte chunks to avoid BLE notification truncation.
// The route is stationary here; transmission adds a short pause between legs.
void sendSummary(const String &report) {
  Serial.print(report);
  if (!USE_BLE || !ble_init_ok) return;
  BLE.poll();
  if (!txChar.subscribed()) return;
  for (unsigned int offset = 0; offset < report.length(); offset += 20) {
    unsigned int count = report.length() - offset;
    if (count > 20) count = 20;
    if (!txChar.subscribed()) break;
    if (!txChar.writeValue((const uint8_t *)report.c_str() + offset, count)) break;
    BLE.poll();
    delay(30);
  }
}

void reportMotion(int leg, bool is_forward, bool is_left, unsigned long left, unsigned long right,
                  float current_yaw, bool gyro_valid, bool gyro_saturated, unsigned long drive_ms,
                  int pwm_left, int pwm_right, int brake_ms, int reverse_ms) {
  float cm_left = left * CM_PER_PULSE;
  float cm_right = right * CM_PER_PULSE;
  float seconds = drive_ms / 1000.0f;
  bool valid_yaw = gyro_valid && !gyro_saturated;
  String report = "\n=== SUMMARY ===\nLEG=" + String(leg) + "\nMODE=" + String(is_forward ? "FORWARD" : (is_left ? "LEFT" : "RIGHT"));
  report += "\nDRIVE_MS=" + String(drive_ms) + " BRAKE_MS=" + String(brake_ms);
  report += "\nBRAKE_REVERSE_MS=" + String(reverse_ms);
  report += "\nPWM_L=" + String(pwm_left) + " PWM_R=" + String(pwm_right);
  report += "\nENC_L=" + String(left) + " ENC_R=" + String(right);
  report += "\nENC_DIFF_L-R=" + String((long)left - (long)right);
  report += "\nREV_L=" + String((float)left / DISK_SLOTS, 3) + " REV_R=" + String((float)right / DISK_SLOTS, 3);
  report += "\nCM_L=" + String(cm_left, 2) + " CM_R=" + String(cm_right, 2);
  report += "\nCM_DIFF_L-R=" + String(cm_left - cm_right, 2);
  report += "\nYAW_DEG=" + (valid_yaw ? String(current_yaw, 2) : String("NA"));
  report += " GYRO=" + String(!has_mpu ? "MISSING" : (!gyro_valid ? "READ_ERROR" : (gyro_saturated ? "SATURATED" : "OK")));
  if (is_forward) {
    report += "\nAVG_WHEEL_CM=" + String((cm_left + cm_right) / 2.0f, 2);
    if (left >= DISK_SLOTS && right >= DISK_SLOTS && left != right) {
      float ratio = left > right ? (float)right / left : (float)left / right;
      int old_pwm = left > right ? pwm_left : pwm_right;
      int suggested = static_cast<int>(old_pwm * ratio);
      report += "\nEST_REDUCE_" + String(left > right ? "L" : "R") + "_PCT=" + String((1.0f - ratio) * 100.0f, 2);
      report += " PWM_DROP=" + String(old_pwm - suggested) + " PWM_TRIAL=" + String(suggested);
      report += "\nEstimate only: assumes equal wheels/no slip; retest. Not applied.";
    } else {
      report += left == right && left >= DISK_SLOTS ? "\nENC balanced: no PWM reduction suggested." : "\nPWM estimate unavailable: need >=20 pulses per wheel.";
    }
  } else {
    report += "\nTURN_MS=" + String(drive_ms);
    report += "\nENC_TURN_DEG_EST=" + String((cm_right - cm_left) / TRACK_WIDTH_CM * 180.0f / 3.14159265f, 2);
    if (valid_yaw && abs(current_yaw) >= 5.0f && seconds > 0) {
      float deg = abs(current_yaw);
      report += "\nAVG_DEG_PER_SEC=" + String(deg / seconds, 2);
      report += "\nEST_MS_90=" + String(seconds * 1000.0f * 90.0f / deg, 0);
      report += " EST_MS_180=" + String(seconds * 1000.0f * 180.0f / deg, 0);
      report += "\nEST_MS_270=" + String(seconds * 1000.0f * 270.0f / deg, 0);
      report += " EST_MS_360=" + String(seconds * 1000.0f * 360.0f / deg, 0);
      report += "\nTime estimates include stopping angle; retest for acceleration/braking.";
    } else {
      report += "\nTurn time estimate unavailable: invalid gyro or angle <5 deg.";
    }
  }
  report += "\nWheel distances are encoder estimates, not lateral drift.\n=== END ===\n";
  sendSummary(report);
}

void reportLeg(int leg, int pwmL, int pwmR) {
  noInterrupts();
  unsigned long spinL = pulse_count_L;
  unsigned long spinR = pulse_count_R;
  interrupts();
  // Snapshot both phases before transmission so BLE delays cannot change the readings.
  float spin_yaw = current_yaw;
  bool spin_valid = gyro_valid;
  bool spin_saturated = gyro_saturated;
  unsigned long spin_drive_ms = last_drive_ms;
  bool is_left = leg == 2 || leg == 4;
  reportMotion(leg, true, false, forward_count_L, forward_count_R, forward_yaw,
               forward_gyro_valid, forward_gyro_saturated, forward_ms, pwmL, pwmR,
               BRAKE_HOLD_MS, emergency_stop ? 0 : FORWARD_BRAKE_REVERSE_MS);
  reportMotion(leg, false, is_left, spinL, spinR, spin_yaw, spin_valid, spin_saturated,
               spin_drive_ms, is_left ? 0 : TURN_SPEED, is_left ? TURN_SPEED : 0,
               BRAKE_HOLD_MS, BRAKE_REVERSE_MS);
}

bool mission_completed = false;

void loop() {
  if (!mission_completed) {
    Serial.println("Place robot at Start Line. Race starting in 3 seconds...");
    delay(500); // 500 ms delay for positioning
    int de_lay = 100;
    forward(LEG1_SPEED_L, LEG1_SPEED_R, 0, 0, LEG1_FORWARD_CM);
    captureForward();
    brake(de_lay); // เบรกค้างทั้งสองล้อก่อนหมุน
    pivotLeftForward(R_SPIN_MS);
    brake(BRAKE_HOLD_MS); // เบรกทั้งสองล้อหลังหมุน ก่อนวิ่งช่วงถัดไป
    reportLeg(1, LEG1_SPEED_L, LEG1_SPEED_R);

    forward(LEG2_SPEED_L, LEG2_SPEED_R, 0, 0, LEG2_FORWARD_CM);
    captureForward();
    brake(de_lay); // เบรกค้างทั้งสองล้อก่อนหมุน
    pivotRightForward(L_SPIN_MS);
    brake(BRAKE_HOLD_MS); // เบรกทั้งสองล้อหลังหมุน ก่อนวิ่งช่วงถัดไป
    reportLeg(2, LEG2_SPEED_L, LEG2_SPEED_R);
    
    forward(LEG3_SPEED_L, LEG3_SPEED_R, 0, 0, LEG3_FORWARD_CM);
    captureForward();
    brake(de_lay); // เบรกค้างทั้งสองล้อก่อนหมุน
    pivotLeftForward(4500);
    brake(BRAKE_HOLD_MS); // เบรกทั้งสองล้อหลังหมุน ก่อนวิ่งช่วงถัดไป
    reportLeg(3, LEG3_SPEED_L, LEG3_SPEED_R);

    forward(LEG4_SPEED_L, LEG4_SPEED_R, 0, 0, LEG4_FORWARD_CM);
    captureForward();
    brake(de_lay); // เบรกค้างทั้งสองล้อก่อนหมุน
    pivotRightForward(L_SPIN_MS);
    brake(BRAKE_HOLD_MS); // เบรกทั้งสองล้อหลังหมุน ก่อนวิ่งช่วงถัดไป
    reportLeg(4, LEG4_SPEED_L, LEG4_SPEED_R);

    forward(LEG5_SPEED_L, LEG5_SPEED_R, 0, 0, LEG5_FORWARD_CM);
    
    brake(BRAKE_HOLD_MS); // Keep the final stop braked after the mission ends.
    mission_completed = true;
    Serial.println("=== Course Completed! Press RESET to run again. ===");
  }
}
