// ============================================================================
// DICE MOVEMENT — สั่งการหุ่นด้วยผลทอยลูกเต๋า 2 ลูก x 3 ครั้ง
// อ้างอิง: /home/poramet/Downloads/arduino_dice_movement_conditions.md
// แยกไฟล์จาก robotcurclerun.ino — คัดลอกค่า pin/calibration ชุดเดียวกัน
// (ฮาร์ดแวร์ตัวเดียวกัน) มาแค่ primitive ที่ใช้จริง (forward/turn) ตัด
// animation/gyro ออกเพื่อให้ไฟล์นี้โฟกัสแค่ logic ลูกเต๋า
// ============================================================================
#include <ArduinoBLE.h>

// ----------------------------------------------------------------------------
// 1. HARDWARE PIN DEFINITIONS (ค่าเดียวกับ robotcurclerun.ino — ยืนยันจาก hardware test)
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
// 2. CALIBRATION (คัดลอกจาก robotcurclerun.ino — ปรับตรงนี้ถ้าฮาร์ดแวร์เปลี่ยน)
// ----------------------------------------------------------------------------
const int DISK_SLOTS = 20;
const float WHEEL_DIAMETER_CM = 6.5;
const float TRACK_WIDTH_CM = 14.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;

int RUN_SPEED = 65000;
int TURN_SPEED = 65535;
int BRAKE_REVERSE_MS = 35;
int BRAKE_HOLD_MS = 150;

float CM_PER_SEC = 86.4;
float DEG_PER_SEC = 310.0;
float ENC_TRIM_L = 1.0;
float TURN_DEG_SCALE = (360.0 / 270.0) * (360.0 / 402.5);
float Kp_enc = 850.0;
float Ki_enc = 20.0;
int LAUNCH_TRIM = 14000;
unsigned long LAUNCH_TRIM_MS = 150;

// 1 ช่องกระดาน = 15 cm (spec ข้อ 1)
const float CM_PER_SLOT = 15.0;

// ----------------------------------------------------------------------------
// 3. ENCODER COUNTERS
// ----------------------------------------------------------------------------
volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;
void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }
void resetEncoders() { pulse_count_L = 0; pulse_count_R = 0; }

// ----------------------------------------------------------------------------
// 4. BLE (NUS) — เดิมจาก robotcurclerun.ino, ใช้ STOP ฉุกเฉินระหว่างวิ่งจริง
// ----------------------------------------------------------------------------
const bool USE_BLE = true;
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 64);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
bool ble_connected = false;
bool emergency_stop = false;
bool start_requested = false;
bool pending_event = false;   // true = ให้รัน runEvent(pending_steps2, pending_steps3)
int pending_steps2 = 0;
int pending_steps3 = 0;

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
        start_requested = true;
        sendTelemetry("[BLE CMD] START RECEIVED!");
      } else if (cmd.startsWith("RUN ")) {
        int sep = cmd.indexOf(' ', 4);
        if (sep > 0) {
          emergency_stop = false;
          pending_steps2 = cmd.substring(4, sep).toInt();
          pending_steps3 = cmd.substring(sep + 1).toInt();
          pending_event = true;
          sendTelemetry("[BLE CMD] RUN " + String(pending_steps2) + " " + String(pending_steps3) + " RECEIVED!");
        }
      }
    }
  } else {
    ble_connected = false;
  }
}

// ----------------------------------------------------------------------------
// 5. MOTOR PRIMITIVES (ตัด animation/gyro ออก เหลือเฉพาะ forward/turn + encoder balance)
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

// วิ่งตรงระยะ distance_cm — จับเวลา (CM_PER_SEC) เป็นตัวหยุดหลัก, LM393 encoder
// (Kp_enc/Ki_enc) คอยแก้สมดุลซ้าย/ขวาสดระหว่างวิ่ง, จบด้วย catch-up phase
// ไล่ pulse ซ้าย/ขวาให้เท่ากันจริงก่อนเบรก (คัดลอกพฤติกรรมจาก robotcurclerun.ino
// forward() ทุกจุด ตัดแค่ animation/gyro ออก)
void forward(float distance_cm) {
  resetEncoders();
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  int base_L = RUN_SPEED;
  int base_R = RUN_SPEED;
  applyMotorSpeeds(base_L, constrain(base_R - LAUNCH_TRIM, 23000, 65535));

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

    int launch_bias = (elapsed < LAUNCH_TRIM_MS) ? LAUNCH_TRIM : 0;
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
  if (labs((long)pulse_count_L - (long)pulse_count_R) > 2) {
    sendTelemetry("[Forward] WARNING: catch-up timeout, L=" + String(pulse_count_L) + " R=" + String(pulse_count_R));
  }

  brake(100);
  stop();
  delay(50);
}

// หมุนขวาอยู่กับที่ target_deg องศา (pivot สองล้อ, encoder-gated)
void turn_right(float target_deg) {
  resetEncoders();
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
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
      sendTelemetry("[Turn Right] WARNING: encoder timeout");
      break;
    }
    processBLE();
    if (emergency_stop) { stop(); return; }
    delay(2);
  }

  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
  applyMotorSpeeds(TURN_SPEED, TURN_SPEED);
  delay(BRAKE_REVERSE_MS);
  brake(BRAKE_HOLD_MS);
  stop();
  delay(100);
}

// หมุนซ้ายอยู่กับที่ target_deg องศา
void turn_left(float target_deg) {
  resetEncoders();
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
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
      sendTelemetry("[Turn Left] WARNING: encoder timeout");
      break;
    }
    processBLE();
    if (emergency_stop) { stop(); return; }
    delay(2);
  }

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
  applyMotorSpeeds(TURN_SPEED, TURN_SPEED);
  delay(BRAKE_REVERSE_MS);
  brake(BRAKE_HOLD_MS);
  stop();
  delay(100);
}

// Single-wheel pivot: ล้อขวาหยุดนิ่ง ล้อซ้ายเดินหน้า — จุดหมุนอยู่ที่ล้อขวา
// (รัศมี TRACK_WIDTH_CM เต็ม ไม่ใช่ครึ่งเดียวแบบ turn_right/turn_left จึงคูณ 2.0)
void pivotLeftForward(float target_deg) {
  if (target_deg <= 0.0) return;
  resetEncoders();
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  applyMotorSpeeds(TURN_SPEED, 0);

  float scaled_deg = target_deg * TURN_DEG_SCALE;
  float arc_cm = (scaled_deg / 360.0) * 2.0 * PI * TRACK_WIDTH_CM;
  unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);
  unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 2.0 * 3.0);
  unsigned long pivot_start = millis();

  while (true) {
    if (pulse_count_L >= target_pulses) break;
    if (millis() - pivot_start >= timeout_ms) {
      sendTelemetry("[Pivot Left Forward] WARNING: encoder timeout");
      break;
    }
    processBLE();
    if (emergency_stop) { stop(); return; }
    delay(2);
  }
  stop();
  delay(100);
}

// Single-wheel pivot: ล้อขวาหยุดนิ่ง ล้อซ้ายถอยหลัง — จุดหมุนอยู่ที่ล้อขวา
void pivotLeftBackward(float target_deg) {
  if (target_deg <= 0.0) return;
  resetEncoders();
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  applyMotorSpeeds(TURN_SPEED, 0);

  float scaled_deg = target_deg * TURN_DEG_SCALE;
  float arc_cm = (scaled_deg / 360.0) * 2.0 * PI * TRACK_WIDTH_CM;
  unsigned long target_pulses = (unsigned long)(arc_cm / CM_PER_PULSE);
  unsigned long timeout_ms = (unsigned long)((scaled_deg / DEG_PER_SEC) * 1000.0 * 2.0 * 3.0);
  unsigned long pivot_start = millis();

  while (true) {
    if (pulse_count_L >= target_pulses) break;
    if (millis() - pivot_start >= timeout_ms) {
      sendTelemetry("[Pivot Left Backward] WARNING: encoder timeout");
      break;
    }
    processBLE();
    if (emergency_stop) { stop(); return; }
    delay(2);
  }
  stop();
  delay(100);
}

// ----------------------------------------------------------------------------
// 6. DICE LOGIC — เวอร์ชันบังคับทิศ (heading คงที่ทุกเหตุการณ์, ไม่ผูกกับผลทอยแล้ว)
// ----------------------------------------------------------------------------
// ทิศเริ่มต้นบังคับคงที่: หมุนขวาเสมอไปจนถึง "ตะวันออกเฉียงใต้" (SE) — อยู่กึ่งกลาง
// ระหว่าง "ขวา"(90°) กับ "หลัง"(180°) ตามมุมเดิมของ spec = 135° หมุนขวาครั้งเดียว
// ตอนเริ่มภารกิจ ไม่ขึ้นกับผลทอยลูกเต๋าครั้งที่ 1 อีกต่อไป (ตัด roll1 ออก)
const float FIXED_HEADING_DEG = 135.0; // SE, หมุนขวาอย่างเดียวเสมอ

int rollDice() { return random(1, 7) + random(1, 7); } // ผลรวม 2 ลูกเต๋า = 2–12

void applyFixedHeading() { turn_right(FIXED_HEADING_DEG); }

// จัดกลุ่มจำนวนช่องที่ทอยได้เป็น 2 เหตุการณ์: ≤8 ช่อง = event 1, >8 ช่อง = event 2
// (เกณฑ์ตามที่กำหนด — ปรับเลข 8 ตรงนี้จุดเดียวถ้าจะเปลี่ยนเกณฑ์)
int classifyStepsEvent(int steps) { return (steps <= 8) ? 1 : 2; }

// รวม event ของครั้งที่ 2 กับ 3 เป็นรหัสเดียว 1-4: (1,1)->1 (1,2)->2 (2,1)->3 (2,2)->4
int combinedEventId(int event2, int event3) { return (event2 - 1) * 2 + event3; }

// จุดหมุนเสริม (single-wheel pivot) ต่อจาก spin หลัก — ค่าเริ่มต้น 0 (ปิด/ข้าม)
// เหมือน TURN_DEG ส่วนใหญ่ใน robotcurclerun.ino, ปรับเป็น >0 ได้ถ้าต้องจูนมุมเพิ่ม
const float PIVOT_CORRECTION_DEG = 0.0;

// เดินตาม leg นั้น — ถ้าช่องที่ทอยได้ leg นี้เกิน 8 (event 2) ให้เลี้ยวขวาเพิ่ม
// (spin turn_right(90) -> pivot เสริม pivotLeftBackward) ก่อนวิ่ง เปลี่ยนทิศจริง
// ให้ event 1/2 ต่างกันทางกายภาพ ไม่ใช่แค่ label; ถ้า ≤8 (event 1) วิ่งตรงตามทิศเดิม
void executeMove(int steps) {
  if (steps > 8) {
    sendTelemetry("[Move] เกิน 8 ช่อง -> เลี้ยวขวาเพิ่ม");
    turn_right(90.0);
    pivotLeftBackward(PIVOT_CORRECTION_DEG);
  }
  if (emergency_stop) return;
  forward(steps * CM_PER_SLOT);
}

// ลำดับการทำงาน: หมุนขวาไป SE ครั้งเดียว -> วิ่งตามช่องที่ทอยได้ครั้งที่ 2 -> ครั้งที่ 3
void processMovement() {
  sendTelemetry("[Heading] หมุนขวาไปทิศตะวันออกเฉียงใต้ (SE, " + String(FIXED_HEADING_DEG, 0) + "°)");
  applyFixedHeading();
  if (emergency_stop) return;

  int sum2 = rollDice();
  int steps2 = sum2;
  int event2 = classifyStepsEvent(steps2);
  sendTelemetry("[Roll 2] sum=" + String(sum2) + " ช่อง -> event " + String(event2));
  executeMove(steps2);
  if (emergency_stop) return;

  int sum3 = rollDice();
  int steps3 = sum3;
  int event3 = classifyStepsEvent(steps3);
  sendTelemetry("[Roll 3] sum=" + String(sum3) + " ช่อง -> event " + String(event3));
  executeMove(steps3);
  if (emergency_stop) return;

  int eventId = combinedEventId(event2, event3);
  sendTelemetry("[Mission] เหตุการณ์รวม #" + String(eventId) + " (event2=" + String(event2) +
                ", event3=" + String(event3) + ") — SE " + String(steps2) + " ช่อง, " +
                String(steps3) + " ช่อง");
}

// เรียกเหตุการณ์เฉพาะเจาะจงด้วยจำนวนช่อง (ครั้งที่ 2, 3) ตรงๆ — ข้าม rollDice()
// ทั้งหมด ใช้ทดสอบซ้ำๆ ได้แน่นอน ทิศยังคงหมุนขวาไป SE ครั้งเดียวเหมือน processMovement()
void runEvent(int steps2, int steps3) {
  if (steps2 < 2 || steps2 > 12 || steps3 < 2 || steps3 > 12) {
    sendTelemetry("[Run] invalid steps (2-12): " + String(steps2) + "," + String(steps3));
    return;
  }
  int event2 = classifyStepsEvent(steps2);
  int event3 = classifyStepsEvent(steps3);
  int eventId = combinedEventId(event2, event3);
  sendTelemetry("[Run Event #" + String(eventId) + "] SE " + String(steps2) + " ช่อง, " +
                String(steps3) + " ช่อง (event2=" + String(event2) + ", event3=" + String(event3) + ")");

  applyFixedHeading();
  if (emergency_stop) return;
  executeMove(steps2);
  if (emergency_stop) return;
  executeMove(steps3);
}

// ----------------------------------------------------------------------------
// 7. SETUP / LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== DICE MOVEMENT READY ===");

  randomSeed(micros());

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
      BLE.setLocalName("DICE-ROBOT");
      BLE.setDeviceName("DICE-ROBOT");
      BLE.setAdvertisedService(uartService);
      uartService.addCharacteristic(txChar);
      uartService.addCharacteristic(rxChar);
      BLE.addService(uartService);
      BLE.advertise();
      Serial.println("[BLE] Online! Device Name: DICE-ROBOT — ส่ง START เพื่อทอยลูกเต๋า");
    } else {
      Serial.println("[BLE] Warning: BLE failed to initialize!");
    }
  }
  Serial.println("ส่ง START ผ่าน BLE หรือพิมพ์ 'start' ใน Serial Monitor เพื่อทอยสุ่มจริง (หมุนขวาไป SE เสมอ)");
  Serial.println("พิมพ์ 'run <steps2> <steps3>' (2-12 ทั้งคู่) หรือ BLE 'RUN <steps2> <steps3>' เพื่อรันเหตุการณ์เฉพาะ");
}

void loop() {
  processBLE();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    if (cmd == "start") {
      emergency_stop = false;
      start_requested = true;
    } else if (cmd == "stop") {
      emergency_stop = true;
      stop();
    } else if (cmd.startsWith("run ")) {
      int sep = cmd.indexOf(' ', 4);
      if (sep > 0) {
        emergency_stop = false;
        pending_steps2 = cmd.substring(4, sep).toInt();
        pending_steps3 = cmd.substring(sep + 1).toInt();
        pending_event = true;
      }
    }
  }

  if (start_requested && !emergency_stop) {
    start_requested = false;
    processMovement();
  } else if (pending_event && !emergency_stop) {
    pending_event = false;
    runEvent(pending_steps2, pending_steps3);
  }
}
