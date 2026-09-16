#include <Wire.h>
#include "Arduino_LED_Matrix.h"
#include "/home/poramet/Documents/Lab_IOT_Robot/animation/animation.h"
#include <ArduinoBLE.h>

// ============================================================================
// 0. สารบัญตัวแปรปรับค่า (TUNING CHEAT SHEET)
// ตัวแปรจริงอยู่ห่างลงไปด้านล่าง (ค้นหาชื่อตัวแปรเพื่อกระโดดไปแก้ค่า)
// อาการ                          -> ตัวแปรที่ต้องปรับ
// ------------------------------------------------------------------
// รถวิ่งตรงเร็ว/ช้าไปโดยรวม        -> RUN_SPEED (ตัวเดียว ใช้ทุก leg)
// เบี้ยวสะสมหลัง spin/turn ของ leg ไหน ดึงกลับแนวเส้นเดิม -> LEG{n}_CORRECTION_DEG
//   (บวก = หักขวา, ลบ = หักซ้าย)
// ตอนปล่อยรถ (millisecond 0) ดึงขวา/ซ้ายแรง ก่อน encoder ทันแก้ -> LAUNCH_TRIM,
//   LAUNCH_TRIM_MS (ลดพลังขวาไว้ชั่วคราวตอนออกตัว; ถ้ากลายเป็นดึงซ้ายให้ลด/ปิด)
// หมุน 360°/เลี้ยว 90° ที่ pivot ช้า/กระตุก -> TURN_SPEED
// วิ่งตรงส่ายไปมา (ไม่นิ่ง)         -> Kp_enc (ลด = นิ่งขึ้นแต่แก้ช้าลง)
// วิ่งตรงเบี้ยวสะสมเป็นเส้นโค้งช้าๆ -> Ki_enc
// หมุน 360°/เลี้ยว 90° ได้มุมไม่ตรง (เกิน/ขาด) -> TURN_DEG_SCALE
//   (มุมจริงเกินที่สั่ง = ลดค่าลง, มุมจริงขาด = เพิ่มค่าขึ้น)
//   turn_right()/turn_left() หยุดตาม pulse encoder แล้ว (ไม่ใช่เวลาอีกต่อไป) —
//   DEG_PER_SEC ตอนนี้ใช้แค่คำนวณ timeout กันค้าง กับยังเป็นตัวหยุดจริงใน
//   pivotLeftForward()/pivotLeftBackward() (ล้อเดียว ยังจับเวลาอยู่)
// วิ่งตรงได้ระยะไม่ตรง (เกิน/ขาด cm ที่สั่ง) -> CM_PER_SEC
//   (ระยะจริงเกิน = เพิ่มค่านี้ขึ้น, ระยะจริงขาด = ลดค่านี้ลง)
// เบรกหลังหยุดกระตุกแรง/เบาไป      -> BRAKE_REVERSE_MS, BRAKE_HOLD_MS
// ระยะที่วิ่ง/หมุน/เลี้ยวแต่ละช่วงของเส้นทาง -> LEG1..LEG5_* (ในบล็อก
//   ROUTE PARAMETERS เหนือ loop())
// ------------------------------------------------------------------

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
// 2026-09-15: สลับกลับเป็น encoder ขา11=ซ้าย, ขา3=ขวา — วิ่งจริงหลังสลับ
// ระยะห่างรอบล้อ (pulse) ซ้าย/ขวาแคบลงเหลือ <100 (ก่อนหน้านี้ห่างไม่หยุดโต
// ทั้งที่ correction ชนเพดานแล้ว อาการตรงกับ pulse/มอเตอร์จับคู่ผิดข้าง) —
// เชื่อผลจริงตรงนี้มากกว่าคอมเมนต์จับคู่เดิมด้านบน (เคย "ยืนยัน" ไว้ผิด)
const int ENCODER_L = 11;
const int ENCODER_R = 3;

// Objects
ArduinoLEDMatrix matrix;

// ============================================================================
// 2. ROBOT PHYSICAL GEOMETRY & CALIBRATION
// ============================================================================
const int DISK_SLOTS = 20;                     // 20 holes on encoder disc (1 revolution)
const float WHEEL_DIAMETER_CM = 6.5;           // Wheel diameter: 6.5 cm
const float TRACK_WIDTH_CM = 14.5;             // Distance between left & right wheels: 14.5 cm
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM; // ~20.42 cm
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;      // ~1.021 cm per pulse

// ============================================================================
// 16-BIT SPEED & BALANCE CONFIGURATION (analogWriteResolution = 16-bit: 0 - 65535)
// ============================================================================
int RUN_SPEED = 65000;   // ความเร็ววิ่งตรง 16-bit จุดสมดุลธรรมชาติล้อซ้าย (เหลือ Headroom ถึง 65535 ให้เร่งแซงได้)
int TURN_SPEED = 65535;  // ความเร็วตอนหมุนเลี้ยว 16-bit สูงสุดเต็มพิกัด 100% (ป้องกันมอเตอร์กระตุกเวลาหมุนบนพื้น)
int BRAKE_REVERSE_MS = 35; // สวนกระแสมอเตอร์เพื่อหยุดแรงเฉื่อยสะบัดทันที (ms)
int BRAKE_HOLD_MS = 150;   // ล็อกล้อด้วยระบบ Dynamic Brake (ms)

// ตัวกำหนดระยะทาง/มุม: เปลี่ยนจากนับ pulse encoder เป็นจับเวลา (delay/millis)
// แทน — วิ่ง/หมุนตามเวลาที่คำนวณไว้ตรงๆ ไม่รอ pulse ถึงเป้าหมายอีกต่อไป
// ประมาณค่าจาก log ฮาร์ดแวร์จริงที่ผ่านมา (ไม่ใช่ทฤษฎีล้วน) แต่ยังหยาบ —
// ความเร็วจริงเปลี่ยนตามแบตเตอรี่/พื้นผิว ต้องคอยเทียบกับระยะจริงแล้วปรับสองค่านี้
// ข้อเสีย: ไม่มี feedback แล้ว ถ้าล้อติด/ลื่นจะไม่รู้ตัว (ต่างจากแบบนับ pulse เดิม)
float CM_PER_SEC  = 86.4;  // ปรับหลังเปลี่ยนมอเตอร์ Hyper Dash 3: สั่ง 150cm ได้ระยะจริง 240cm
                            // (เร็วกว่าค่าประเมินเดิม 54.0 อยู่ 1.6 เท่า -> 54.0*1.6=86.4)
float DEG_PER_SEC = 310.0; // ความเร็วหมุน/เลี้ยวโดยประมาณที่ TURN_SPEED (จาก log จริง: 360° ≈ 1.15 วิ)

// Motor Balance Tuning 16-bit
// MOTOR_L_RATIO/MOTOR_R_RATIO ถูกเอาออก — ทุก leg ใช้ RUN_SPEED ตัวเดียวกัน
// ตรงๆ (ไม่มี LEG{n}_SPEED_L/R แยกแล้ว) สมดุลซ้าย/ขวาปล่อยให้ LM393 encoder
// (Kp_enc/Ki_enc, ดูด้านล่าง) แก้สดระหว่างวิ่งแทน
float ENC_TRIM_L = 1.0;      // ตัวคูณค่า pulse ซ้ายก่อนเข้าสูตร error (ไม่กระทบ PWM จริง แค่การอ่านค่า)
float TURN_DEG_SCALE = (360.0 / 270.0) * (360.0 / 402.5); // ตัวชดเชยมุมหมุน/เลี้ยว — เพิ่ม/ลดถ้ามุมจริงขาด/เกิน (ยืนยันจาก hardware test ≈ 1.19255)
float Kp_enc = 850.0;        // ความไวของ loop แก้สมดุลล้อ (จาก encoder) — สูง=ตอบสนองไว แต่ส่ายง่าย
                              // ลดจาก 1100 (รู้สึกคูณ error แรงไป กระชากเร็วเกินตั้งแต่ diff ยังน้อย) —
                              // ถ้าไปต่อแล้วยังห่างไม่ปิด ปัญหาน่าจะเป็นฮาร์ดแวร์ชนเพดาน ไม่ใช่ gain แล้ว
float Ki_enc = 20.0;         // ตัวสะสม error ระยะยาว (จาก encoder) — แก้อาการเบี้ยวสะสมเป็นเส้นโค้ง
                              // ลดจาก 25 คู่กับ Kp_enc ด้านบน ด้วยเหตุผลเดียวกัน
int LAUNCH_TRIM = 14000;     // ตอนออกตัว (millisecond 0) pulse ยังไม่ทันสะสมพอให้ Kp_enc/Ki_enc
                              // แก้ทัน จึงลดพลังล้อขวาลงชั่วคราวกันเบี้ยวขวาแรงตอนปล่อย (จากอาการจริงที่รายงาน)
                              // ปรับขึ้นถ้ายังดึงขวา, ลดลง/เป็น 0 ถ้ากลายเป็นดึงซ้ายแทน
unsigned long LAUNCH_TRIM_MS = 150; // ระยะเวลาที่ใช้ LAUNCH_TRIM ก่อนปล่อยให้ Kp_enc/Ki_enc คุมเต็มที่
float Kp_gyro = 900.0;       // ความไวของ loop แก้สมดุล (จาก gyro, ใช้เมื่อ USE_MPU6050 = true)

// ============================================================================
// ROUTE PARAMETERS — ค่าแยกอิสระต่อจุด ปรับจุดไหนไม่กระทบจุดอื่น
// ============================================================================
// ความเร็ววิ่งตรงใช้ RUN_SPEED ตัวเดียวกันทุก leg (มอเตอร์ตัวเดียวกัน ไม่มี
// SPEED_L/SPEED_R แยกราย leg อีกต่อไป) — ระหว่างวิ่ง LM393 encoder ซ้าย/ขวา
// (ผ่าน Kp_enc/Ki_enc ใน forward()) คอยวัด pulse แล้วแก้สมดุลสองล้อสด ๆ ให้
// วิ่งตรงเอง ไม่ต้องตั้งความเร็วต่างกันรายล้อ/ราย leg
//
// LEG{n}_CORRECTION_DEG: มุมหักกลับเพื่อดึงรถกลับมาแนวเส้นทางเดิมถ้าเบี้ยว
// สะสม — ค่าบวก = หักขวา (เรียก turn_right), ค่าลบ = หักซ้าย (เรียก
// turn_left), 0 = ไม่หัก (ค่าเริ่มต้น ต้องจูนจากการวิ่งจริงเทียบเส้นทางที่ควรได้)
// LEG{n}_CORRECTION_AT_CM: ระยะ (cm) นับจากต้น leg ที่จะหัก "ระหว่างวิ่งตรง"
// — 0 = ปิด (ไม่หักกลางทาง, ถ้า CORRECTION_DEG ไม่ใช่ 0 จะหักหลัง spin/turn
// ของ leg จบแทนเหมือนเดิม), >0 = หยุดวิ่งตรงที่ระยะนี้ หักมุม แล้ววิ่งต่อจน
// ครบ FORWARD_CM (ต้องน้อยกว่า FORWARD_CM ของ leg เดียวกัน)
const float LEG1_FORWARD_CM = 590;
const float LEG1_SPIN_DEG   = 345; // spin right
const float LEG1_TURN_DEG   = 0.0;  // turn left
const float LEG1_CORRECTION_DEG = 0.0;
const float LEG1_CORRECTION_AT_CM = 0.0;

const float LEG2_FORWARD_CM = 800.0;
const float LEG2_SPIN_DEG   = 494; // spin left
const float LEG2_TURN_DEG   = 0;  // turn right
const float LEG2_CORRECTION_DEG = 2.0; // ออกขวาตอนวิ่งผ่าน 750cm หักซ้ายชดเชย
const float LEG2_CORRECTION_AT_CM = 0.0; // หักกลางทางที่ 750cm ไม่ใช่รอจบ leg

const float LEG3_FORWARD_CM = 590.0;
const float LEG3_SPIN_DEG   = 345; // spin right
const float LEG3_TURN_DEG   = 0.0;  // turn left
const float LEG3_CORRECTION_DEG = -2;
const float LEG3_CORRECTION_AT_CM = 0.0;
const float LEG3_CORRECTION2_DEG = 10.0;
const float LEG3_CORRECTION2_AT_CM = 350.0;

const float LEG4_FORWARD_CM = 820.0;
const float LEG4_SPIN_DEG   = 645.0; // spin left
const float LEG4_TURN_DEG   = 0.0;  // turn right
const float LEG4_CORRECTION_DEG = -20.0;
const float LEG4_CORRECTION_AT_CM = 10.0;
const float LEG4_CORRECTION2_DEG = 20.0;
const float LEG4_CORRECTION2_AT_CM = 250.0;

const float LEG5_FORWARD_CM = 0;
const float LEG5_SPIN_DEG   = 0.0; // spin right, then stop

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

float getRevolutionsL() { return (float)pulse_count_L / DISK_SLOTS; }
float getRevolutionsR() { return (float)pulse_count_R / DISK_SLOTS; }

// ============================================================================
// 4. BLUETOOTH LOW ENERGY (BLE) - NORDIC UART SERVICE (iOS & Android)
// ============================================================================
const bool USE_BLE = true; // เปิดใช้งาน Bluetooth ไร้สายสำหรับ iPhone
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 64);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
bool ble_connected = false;
bool emergency_stop = false;

void sendTelemetry(const String &msg) {
  Serial.println(msg);
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
// 4.1 MPU-6050 GYROSCOPE DRIVER (Auto-Detect & Fallback)
// ============================================================================
// Set to true only when MPU-6050 is physically wired to A4(SDA) and A5(SCL)
// Set to false when using Encoders only (prevents I2C hanging on floating pins!)
const bool USE_MPU6050 = false;

const int MPU_ADDR = 0x68;
bool has_mpu = false;
float gyro_z_offset = 0;
float current_yaw = 0;
unsigned long last_gyro_time = 0;

bool initMPU6050() {
  if (!USE_MPU6050) {
    Serial.println("[IMU] MPU-6050 disabled (USE_MPU6050 = false). Using Encoders for heading & turning.");
    has_mpu = false;
    return false;
  }

  // If enabled, turn on internal pullups and timeout to prevent hanging on floating lines
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  Wire.setWireTimeout(10000, true); // 10ms timeout with auto-reset to prevent lockup
  #endif

  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("[IMU] MPU-6050 not detected. Using Encoders for heading & turning.");
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
  Serial.println("[IMU] Calibration complete! Gyroscope Heading Lock is ACTIVE.");
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
    float rate_z = (raw_z - gyro_z_offset) / 131.0; // 131 LSB/(deg/s)
    if (abs(rate_z) > 0.15) { // Filter sensor noise
      current_yaw += rate_z * dt;
    }
  }
}

void resetYaw() {
  current_yaw = 0;
  last_gyro_time = micros();
}

// ============================================================================
// 5. MOTOR CONTROL PRIMITIVES
// ============================================================================
void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

// Active Electronic Brake (L298N Dynamic Brake: short-circuits coils to lock wheels rigid)
void brake(int duration_ms = 150) {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 1);
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
// 6. HIGH-SPEED NAVIGATION APIs
// ============================================================================

// Forward by distance with Instant Launch & Heading Lock
void forward(int speed_motorL, int speed_motorR, int B_L = 0, int B_R = 0, float distance_cm = 0) {
  resetEncoders();
  resetYaw();

  // Set motor directions forward
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  int base_L = (speed_motorL - B_R);
  int base_R = (speed_motorR - B_L);

  // Instant Launch: Apply full power simultaneously from millisecond 0!
  // ลดพลังขวาไว้ก่อนด้วย LAUNCH_TRIM (pulse ยังไม่ทันสะสมให้ Kp_enc/Ki_enc แก้)
  applyMotorSpeeds(base_L, constrain(base_R - LAUNCH_TRIM, 23000, 65535));

  // If distance is 0 or not specified, run one single animation cycle (default mode)
  if (distance_cm <= 0) {
    int total_frames = sizeof(walk) / sizeof(walk[0]);
    for (int i = 0; i < total_frames; i++) {
      updateYaw();
      long error = has_mpu ? (long)(current_yaw * 5.0) : ((long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R);
      int adjustment = constrain((int)(Kp_enc * error), -40000, 40000);
      applyMotorSpeeds(constrain(base_L - adjustment, 23000, 65535), constrain(base_R + adjustment, 23000, 65535));
      loadFlippedYFrame(walk[i]);
      delay(walk[i][3]);
    }
    return;
  }

  // Running for specified distance (cm) — จับเวลาแทนนับ pulse
  unsigned long target_duration_ms = (unsigned long)(distance_cm / CM_PER_SEC * 1000.0);
  unsigned long drive_start = millis();
  int total_frames = sizeof(walk) / sizeof(walk[0]);
  int anim_frame = 0;
  unsigned long last_anim_time = millis();
  unsigned long last_print_time = millis();
  long integral_error = 0;

  while (true) {
    unsigned long elapsed = millis() - drive_start;
    if (elapsed >= target_duration_ms) break;

    updateYaw();

    // Closed-loop Heading Lock: Gyro (Yaw) if available, otherwise Encoders
    long error;
    int adjustment;
    if (has_mpu) {
      error = (long)(current_yaw * 5.0); // Heading error in tenths of degrees
      adjustment = constrain((int)(Kp_gyro * current_yaw), -18000, 18000);
    } else {
      error = (long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R;
      integral_error = constrain(integral_error + error, -800, 800);
      adjustment = constrain((int)(Kp_enc * error + Ki_enc * integral_error), -40000, 40000);
    }

    // ช่วง LAUNCH_TRIM_MS แรก ยังลดพลังขวาไว้กันเบี้ยวขวาตอนปล่อย เหมือนตอน
    // instant launch — ค่อยๆ คลายให้ Kp_enc/Ki_enc คุมเต็มที่หลังพ้นช่วงนี้
    int launch_bias = (elapsed < LAUNCH_TRIM_MS) ? LAUNCH_TRIM : 0;

    int current_L = constrain(base_L - adjustment, 23000, 65535);
    int current_R = constrain(base_R + adjustment - launch_bias, 23000, 65535);
    applyMotorSpeeds(current_L, current_R);

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

    // Telemetry print every 300 ms (Outputs to both USB Serial and Bluetooth BLE)
    if (millis() - last_print_time >= 300) {
      String t = " L: " + String(pulse_count_L) + " | R: " + String(pulse_count_R) + " | PWM: " + String(current_L) + "/" + String(current_R);
      sendTelemetry(t);
      last_print_time = millis();
    }

    delay(10);
  }

  // Catch-up: จบเวลาที่กำหนดแล้ว แต่รอบวิ่ง (pulse) ซ้าย/ขวาอาจยังไม่เท่ากัน —
  // ขับเฉพาะล้อที่ตามหลังต่อสั้นๆ (อีกล้อหยุด) จนกว่า pulse ซ้าย=ขวา หรือ
  // timeout กันค้าง (300ms) กันเบี้ยวตกค้างก่อนเบรก
  unsigned long catchup_start = millis();
  while (!emergency_stop && labs((long)pulse_count_L - (long)pulse_count_R) > 2 && millis() - catchup_start < 300) {
    if (pulse_count_L < pulse_count_R) {
      applyMotorSpeeds(base_L, 0);
    } else {
      applyMotorSpeeds(0, base_R);
    }
    delay(2);
  }
  if (labs((long)pulse_count_L - (long)pulse_count_R) > 2) {
    sendTelemetry("[Forward] WARNING: catch-up timeout, L=" + String(pulse_count_L) + " R=" + String(pulse_count_R));
  }

  // Active Brake on arrival
  brake(100);
  stop();
  delay(50); // Settle
}

// ฟังก์ชันเรียกใช้งานง่าย: ใส่แค่ระยะทาง (cm) ระบบใช้ความเร็ว RUN_SPEED อัตโนมัติ
void forward(float distance_cm = 0) {
  forward(RUN_SPEED, RUN_SPEED, 0, 0, distance_cm);
}

// Same as forward(distance_cm), but once the robot has covered boost_at_cm,
// bumps the left motor's base PWM by boost_ratio for the remaining distance.
// Closed-loop heading correction (Kp_enc/Ki_enc) still runs on top of this.
void forwardBoostLeftAfter(float distance_cm, float boost_at_cm, float boost_ratio) {
  resetEncoders();
  resetYaw();

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  int base_L = RUN_SPEED;
  int base_R = RUN_SPEED;
  bool boosted = false;

  applyMotorSpeeds(base_L, base_R);

  unsigned long target_duration_ms = (unsigned long)(distance_cm / CM_PER_SEC * 1000.0);
  unsigned long boost_at_ms = (unsigned long)(boost_at_cm / CM_PER_SEC * 1000.0);
  unsigned long drive_start = millis();
  int total_frames = sizeof(walk) / sizeof(walk[0]);
  int anim_frame = 0;
  unsigned long last_anim_time = millis();
  unsigned long last_print_time = millis();
  long integral_error = 0;

  while (true) {
    unsigned long elapsed = millis() - drive_start;
    if (elapsed >= target_duration_ms) break;

    if (!boosted && elapsed >= boost_at_ms) {
      base_L = (int)(base_L * boost_ratio);
      boosted = true;
      sendTelemetry("[Forward+Boost] Left boosted at " + String(elapsed) + " ms");
    }

    updateYaw();

    long error;
    int adjustment;
    if (has_mpu) {
      error = (long)(current_yaw * 5.0);
      adjustment = constrain((int)(Kp_gyro * current_yaw), -18000, 18000);
    } else {
      error = (long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R;
      integral_error = constrain(integral_error + error, -800, 800);
      adjustment = constrain((int)(Kp_enc * error + Ki_enc * integral_error), -18000, 18000);
    }

    int current_L = constrain(base_L - adjustment, 23000, 65535);
    int current_R = constrain(base_R + adjustment, 23000, 65535);
    applyMotorSpeeds(current_L, current_R);

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

    if (millis() - last_print_time >= 300) {
      String t = " L: " + String(pulse_count_L) + " | R: " + String(pulse_count_R) + " | PWM: " + String(current_L) + "/" + String(current_R);
      sendTelemetry(t);
      last_print_time = millis();
    }

    delay(10);
  }

  brake(100);
  stop();
  delay(50);
}

// Backward by distance
void backward(int speed_motorL, int speed_motorR, int B_L = 0, int B_R = 0, float distance_cm = 0) {
  resetEncoders();
  resetYaw();

  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

  int base_L = (speed_motorL - B_R);
  int base_R = (speed_motorR - B_L);

  applyMotorSpeeds(base_L, base_R);

  if (distance_cm <= 0) {
    int total_frames = sizeof(walk) / sizeof(walk[0]);
    for (int i = 0; i < total_frames; i++) {
      updateYaw();
      long error = has_mpu ? (long)(current_yaw * 5.0) : ((long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R);
      int adjustment = constrain((int)(Kp_enc * error), -40000, 40000);
      applyMotorSpeeds(constrain(base_L - adjustment, 23000, 65535), constrain(base_R + adjustment, 23000, 65535));
      loadFlippedXFrame(walk[i]);
      delay(walk[i][3]);
    }
    return;
  }

  unsigned long target_duration_ms = (unsigned long)(distance_cm / CM_PER_SEC * 1000.0);
  unsigned long drive_start = millis();
  while (millis() - drive_start < target_duration_ms) {
    updateYaw();
    long error = has_mpu ? (long)(current_yaw * 5.0) : ((long)(pulse_count_L * ENC_TRIM_L) - (long)pulse_count_R);
    int adjustment = constrain((int)(Kp_enc * error), -40000, 40000);
    applyMotorSpeeds(constrain(base_L - adjustment, 23000, 65535), constrain(base_R + adjustment, 23000, 65535));
    delay(10);
  }
  brake(100);
  stop();
  delay(50);
}

void backward(float distance_cm = 0) {
  backward(RUN_SPEED, RUN_SPEED, 0, 0, distance_cm);
}

// Pivot Turn Right by exact degrees (e.g. 90.0)
void turn_right(float target_deg = 90.0, int speed = -1) {
  if (speed <= 0) speed = TURN_SPEED;
  resetEncoders();
  resetYaw();

  // Left forward, Right backward
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

  applyMotorSpeeds(speed, speed); // จ่ายไฟเต็มพิกัดสูงสุด 65535 ทั้งสองล้อ ไม่กระตุก

  if (has_mpu) {
    // Gyroscope mode: wait until yaw angle reaches target
    while (true) {
      updateYaw();
      if (abs(current_yaw) >= target_deg) break;
      delay(2);
    }
  } else {
    // Encoder-based stop: มุมที่สั่ง -> ระยะอาร์กที่ล้อต้องวิ่ง -> จำนวน pulse เป้าหมาย
    // (คูณ TURN_DEG_SCALE ชดเชยแรงหนืดล้ออิสระที่ล็อกไว้ เหมือนเดิม)
    float scaled_deg = target_deg * TURN_DEG_SCALE;
    float arc_cm = (scaled_deg / 360.0) * PI * TRACK_WIDTH_CM;
    unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);

    // Timeout กันค้าง ถ้า encoder ไม่ขยับ/ล้อติด (ประมาณจากเวลาที่ควรใช้ x3)
    unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 3.0);
    unsigned long turn_start = millis();
    long integral_error = 0;

    while (true) {
      unsigned long avg_pulses = (pulse_count_L + pulse_count_R) / 2;
      if (avg_pulses >= target_pulses) break;
      if (millis() - turn_start >= timeout_ms) {
        sendTelemetry("[Turn Right] WARNING: encoder timeout, pulses=" + String(avg_pulses) + "/" + String(target_pulses));
        break;
      }

      // Live L/R balance correction: encoder เดิมใช้แค่จุดหยุด ความเร็วนิ่งตลอด
      // ทำให้ถ้าล้อใดหมุนเร็ว/ช้ากว่าอีกข้าง จุด pivot เบี้ยวได้ทั้งที่ average
      // pulse ถึงเป้า — เพิ่ม correction ให้ pulse ซ้าย/ขวาสมดุลกันระหว่างหมุน
      long error = (long)pulse_count_L - (long)pulse_count_R;
      integral_error = constrain(integral_error + error, -800, 800);
      int adjustment = constrain((int)(Kp_enc * error + Ki_enc * integral_error), -40000, 40000);
      int current_L = constrain(speed - adjustment, 20000, 65535);
      int current_R = constrain(speed + adjustment, 20000, 65535);
      applyMotorSpeeds(current_L, current_R);

      delay(2);
    }
  }

  // --- ACTIVE BRAKE & LOCK WHEELS ---
  // 1. สวนกระแสมอเตอร์สั้นๆ (Counter-torque) เพื่อหยุดแรงเฉื่อยการหมุนทันที ไม่ให้แฉลบเลยจุด
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  applyMotorSpeeds(speed, speed);
  delay(BRAKE_REVERSE_MS);

  // 2. ล็อกล้อด้วยระบบ Dynamic Brake (ชอร์ตขั้วมอเตอร์ตรึงล้อให้อยู่นิ่งสนิท)
  brake(BRAKE_HOLD_MS);
  stop();
  sendTelemetry("[Turn Right] Target: " + String(target_deg, 1) + " deg | Pulses: " + String((pulse_count_L + pulse_count_R) / 2));
  delay(100); // Settle pause
}

// Pivot Turn Left by exact degrees (e.g. 90.0)
void turn_left(float target_deg = 90.0, int speed = -1) {
  if (speed <= 0) speed = TURN_SPEED;
  resetEncoders();
  resetYaw();

  // Left backward, Right forward
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  applyMotorSpeeds(speed, speed); // จ่ายไฟเต็มพิกัดสูงสุด 65535 ทั้งสองล้อ ไม่กระตุก

  if (has_mpu) {
    while (true) {
      updateYaw();
      if (abs(current_yaw) >= target_deg) break;
      delay(2);
    }
  } else {
    // Encoder-based stop: มุมที่สั่ง -> ระยะอาร์กที่ล้อต้องวิ่ง -> จำนวน pulse เป้าหมาย
    // (คูณ TURN_DEG_SCALE ชดเชยแรงหนืดล้ออิสระที่ล็อกไว้ เหมือนเดิม)
    float scaled_deg = target_deg * TURN_DEG_SCALE;
    float arc_cm = (scaled_deg / 360.0) * PI * TRACK_WIDTH_CM;
    unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);

    // Timeout กันค้าง ถ้า encoder ไม่ขยับ/ล้อติด (ประมาณจากเวลาที่ควรใช้ x3)
    unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 3.0);
    unsigned long turn_start = millis();
    long integral_error = 0;

    while (true) {
      unsigned long avg_pulses = (pulse_count_L + pulse_count_R) / 2;
      if (avg_pulses >= target_pulses) break;
      if (millis() - turn_start >= timeout_ms) {
        sendTelemetry("[Turn Left] WARNING: encoder timeout, pulses=" + String(avg_pulses) + "/" + String(target_pulses));
        break;
      }

      // Live L/R balance correction (ดู turn_right() สำหรับเหตุผล)
      long error = (long)pulse_count_L - (long)pulse_count_R;
      integral_error = constrain(integral_error + error, -800, 800);
      int adjustment = constrain((int)(Kp_enc * error + Ki_enc * integral_error), -40000, 40000);
      int current_L = constrain(speed - adjustment, 20000, 65535);
      int current_R = constrain(speed + adjustment, 20000, 65535);
      applyMotorSpeeds(current_L, current_R);

      delay(2);
    }
  }

  // --- ACTIVE BRAKE & LOCK WHEELS ---
  // 1. สวนกระแสมอเตอร์สั้นๆ (Counter-torque) เพื่อหยุดแรงเฉื่อยการหมุนทันที
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  applyMotorSpeeds(speed, speed);
  delay(BRAKE_REVERSE_MS);

  // 2. ล็อกล้อด้วยระบบ Dynamic Brake
  brake(BRAKE_HOLD_MS);
  stop();
  sendTelemetry("[Turn Left] Target: " + String(target_deg, 1) + " deg | Pulses: " + String((pulse_count_L + pulse_count_R) / 2));
  delay(100);
}

// Single-wheel pivot: right wheel stays stationary, left wheel drives backward.
// Pivot point is the (stationary) right wheel, so the left wheel travels along
// a circle of radius TRACK_WIDTH_CM (not TRACK_WIDTH_CM/2 like the two-wheel
// turn_right()/turn_left() pivots) — full-circle arc length is 2x theirs.
// TURN_DEG_SCALE was calibrated for the two-wheel pivot's tire scrub, reused
// here as a starting estimate; may need its own calibration on hardware.
void pivotLeftBackward(float target_deg = 90.0, int speed = -1) {
  if (speed <= 0) speed = TURN_SPEED;
  resetEncoders();
  resetYaw();

  // Left backward, right stationary
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);

  applyMotorSpeeds(speed, 0);

  // Encoder-based stop: ล้อเดียวหมุนกวาดอาร์กรัศมี TRACK_WIDTH_CM เต็ม (ไม่ใช่
  // TRACK_WIDTH_CM/2 แบบ pivot สองล้อ) จึงยาวเป็น 2 เท่าของ turn_right/turn_left
  // ที่มุมเดียวกัน (คูณ 2.0) — หยุดตาม pulse_count_L จริง แทนการจับเวลาล้วนๆ
  float scaled_deg = target_deg * TURN_DEG_SCALE;
  float arc_cm = (scaled_deg / 360.0) * 2.0 * PI * TRACK_WIDTH_CM;
  unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);

  // Timeout กันค้าง (ประมาณจากเวลาที่ควรใช้ x3)
  unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 2.0 * 3.0);
  unsigned long pivot_start = millis();

  while (true) {
    if (pulse_count_L >= target_pulses) break;
    if (millis() - pivot_start >= timeout_ms) {
      sendTelemetry("[Pivot Left Backward] WARNING: encoder timeout, pulse_count_L=" + String(pulse_count_L) + "/" + String(target_pulses));
      break;
    }
    delay(2);
  }

  stop();
  if (pulse_count_L < 3) {
    sendTelemetry("[Pivot Left Backward] WARNING: pulse_count_L=" + String(pulse_count_L) + " — ล้อซ้ายอาจไม่หมุน");
  }
  sendTelemetry("[Pivot Left Backward] Target: " + String(target_deg, 1) + " deg | Pulses: " + String(pulse_count_L));
  delay(100);
}

// Single-wheel pivot: right wheel stays stationary, left wheel drives forward.
// Same geometry as pivotLeftBackward() (2x arc length vs a two-wheel pivot),
// just the opposite direction on the left wheel.
void pivotLeftForward(float target_deg = 90.0, int speed = -1) {
  if (speed <= 0) speed = TURN_SPEED;
  resetEncoders();
  resetYaw();

  // Left forward, right stationary
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);

  applyMotorSpeeds(speed, 0);

  // Encoder-based stop (ดู pivotLeftBackward() สำหรับเหตุผล)
  float scaled_deg = target_deg * TURN_DEG_SCALE;
  float arc_cm = (scaled_deg / 360.0) * 2.0 * PI * TRACK_WIDTH_CM;
  unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);

  unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 2.0 * 3.0);
  unsigned long pivot_start = millis();

  while (true) {
    if (pulse_count_L >= target_pulses) break;
    if (millis() - pivot_start >= timeout_ms) {
      sendTelemetry("[Pivot Left Forward] WARNING: encoder timeout, pulse_count_L=" + String(pulse_count_L) + "/" + String(target_pulses));
      break;
    }
    delay(2);
  }

  stop();
  if (pulse_count_L < 3) {
    sendTelemetry("[Pivot Left Forward] WARNING: pulse_count_L=" + String(pulse_count_L) + " — ล้อซ้ายอาจไม่หมุน");
  }
  sendTelemetry("[Pivot Left Forward] Target: " + String(target_deg, 1) + " deg | Pulses: " + String(pulse_count_L));
  delay(100);
}

// หักมุมกลับหลังจบ leg เพื่อดึงรถกลับแนวเส้นทางเดิม ถ้า spin/turn ของ leg
// นั้นเบี้ยวสะสม — deg บวก = หักขวา (turn_right), ลบ = หักซ้าย (turn_left),
// 0 = ข้าม (ไม่หัก) ใช้คู่กับ LEG{n}_CORRECTION_DEG
void correctHeading(float deg) {
  if (deg == 0.0) return;
  if (deg > 0) {
    Serial.println("Correction: turn right " + String(deg, 1) + " deg");
    turn_right(deg);
  } else {
    Serial.println("Correction: turn left " + String(-deg, 1) + " deg");
    turn_left(-deg);
  }
}

// วิ่งตรงระยะ forward_cm ของ leg นั้น — ถ้า at_cm ระบุไว้ (>0 และน้อยกว่า
// forward_cm) จะหยุดวิ่งตรงที่ระยะ at_cm หักมุม correction_deg กลางทางก่อน
// (ใช้ correctHeading()) แล้ววิ่งต่อจนครบระยะที่เหลือ — ถ้า at_cm<=0 วิ่ง
// ตรงยาวเดียวตามปกติ ไม่หักกลางทาง (ใช้คู่กับ LEG{n}_CORRECTION_AT_CM)
void forwardLegWithCorrection(float forward_cm, float correction_deg, float at_cm) {
  if (correction_deg != 0.0 && at_cm > 0 && at_cm < forward_cm) {
    forward(RUN_SPEED, RUN_SPEED, 0, 0, at_cm);
    correctHeading(correction_deg);
    forward(RUN_SPEED, RUN_SPEED, 0, 0, forward_cm - at_cm);
  } else {
    forward(RUN_SPEED, RUN_SPEED, 0, 0, forward_cm);
  }
}

// เหมือน forwardLegWithCorrection() แต่หักมุมกลางทางได้หลายจุด — at_cm[]/deg[]
// คู่กันตามลำดับ ต้องเรียง at_cm[] จากน้อยไปมาก, count = จำนวนจุดหัก
void forwardLegWithCorrections(float forward_cm, const float at_cm[], const float deg[], int count) {
  float traveled = 0;
  for (int i = 0; i < count; i++) {
    if (deg[i] == 0.0 || at_cm[i] <= traveled || at_cm[i] >= forward_cm) continue;
    forward(RUN_SPEED, RUN_SPEED, 0, 0, at_cm[i] - traveled);
    correctHeading(deg[i]);
    traveled = at_cm[i];
  }
  forward(RUN_SPEED, RUN_SPEED, 0, 0, forward_cm - traveled);
}

// ============================================================================
// 7. LED MATRIX DISPLAY TRANSFORM HELPERS
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

void loadFlippedXFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, true, false);
}

void loadFlippedYFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, false, true);
}

void loadFlippedXYFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, true, true);
}

// ============================================================================
// 8. SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  delay(1000); // 1-second delay for USB Serial enumeration & connection

  Serial.println();
  Serial.println("=========================================");
  Serial.println("   IOT ROBOT SYSTEM ONLINE (COM PORT OK) ");
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

  // Encoders
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);

  // Initialize MPU-6050
  initMPU6050();

  // Initialize Bluetooth Low Energy (BLE)
  if (USE_BLE) {
    if (BLE.begin()) {
      BLE.setLocalName("IOT-ROBOT");
      BLE.setDeviceName("IOT-ROBOT");
      BLE.setAdvertisedService(uartService);
      uartService.addCharacteristic(txChar);
      uartService.addCharacteristic(rxChar);
      BLE.addService(uartService);
      BLE.advertise();
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
bool mission_completed = false;


void loop() {
  if (!mission_completed) {
    Serial.println("Place robot at Start Line. Race starting in 3 seconds...");
    delay(500); // 1-second delay for positioning

    Serial.println("Leg 1: Forward");
    forwardLegWithCorrection(LEG1_FORWARD_CM, LEG1_CORRECTION_DEG, LEG1_CORRECTION_AT_CM);
    Serial.println("Turn Left");
    turn_left(LEG1_SPIN_DEG);
    Serial.println("Pivot Left Forward (right wheel stopped)");
    pivotLeftForward(LEG1_TURN_DEG);
    if (LEG1_CORRECTION_AT_CM <= 0) correctHeading(LEG1_CORRECTION_DEG);

    Serial.println("Leg 2: Forward");
    forwardLegWithCorrection(LEG2_FORWARD_CM, LEG2_CORRECTION_DEG, LEG2_CORRECTION_AT_CM);
    Serial.println("Pivot Left Backward (right wheel stopped)");
    turn_right(LEG2_SPIN_DEG);
    Serial.println("Turn Right");
    pivotLeftBackward(LEG2_TURN_DEG);
    if (LEG2_CORRECTION_AT_CM <= 0) correctHeading(LEG2_CORRECTION_DEG);

    Serial.println("Leg 3: Forward");
    {
      // เรียงจากน้อยไปมาก: 10cm, 350cm
      float leg3_at_cm[] = { LEG3_CORRECTION_AT_CM,  LEG3_CORRECTION2_AT_CM };
      float leg3_deg[]   = { LEG3_CORRECTION_DEG,  LEG3_CORRECTION2_DEG };
      forwardLegWithCorrections(LEG3_FORWARD_CM, leg3_at_cm, leg3_deg, 2);
    }
    Serial.println("Pivot Left Forward (right wheel stopped)");
    turn_left(LEG3_SPIN_DEG);
    Serial.println("Turn Left");
    pivotLeftForward(LEG3_TURN_DEG);
    if (LEG3_CORRECTION_AT_CM <= 0) correctHeading(LEG3_CORRECTION_DEG);

    Serial.println("Leg 4: Forward");
    {
      float leg4_at_cm[] = { LEG4_CORRECTION_AT_CM, LEG4_CORRECTION2_AT_CM };
      float leg4_deg[]   = { LEG4_CORRECTION_DEG, LEG4_CORRECTION2_DEG };
      forwardLegWithCorrections(LEG4_FORWARD_CM, leg4_at_cm, leg4_deg, 2);
    }
    Serial.println("Pivot Left Backward (right wheel stopped)");
    turn_right(LEG4_SPIN_DEG);
    Serial.println("Turn Right");
    pivotLeftBackward(LEG4_TURN_DEG);
    if (LEG4_CORRECTION_AT_CM <= 0) correctHeading(LEG4_CORRECTION_DEG);

    Serial.println("Leg 5: Forward");
    forward(0, 0, 0, 0, LEG5_FORWARD_CM);
    Serial.println("Spin Right");
    turn_right(LEG5_SPIN_DEG);

    stop();
    mission_completed = true;
    Serial.println("=== Course Completed! Press RESET to run again. ===");
  }
}
