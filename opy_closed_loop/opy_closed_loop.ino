// ============================================================================
// OPY CLOSED-LOOP SPEED CONTROLLER (ระบบควบคุมความเร็วคงที่อัตโนมัติ)
// ควบคุมความเร็วต่ำสุดให้คงที่ตลอดเวลา ไม่ว่าแรงดันถ่าน/แบตเตอรี่จะเต็มหรืออ่อน
// ใช้หลักการ Trajectory Tracking + Closed-Loop PI Control ร่วมกับ LM393 Encoders
// ============================================================================
#include <Arduino.h>

// --- กำหนดขามอเตอร์ ---
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;

const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

// --- LM393 Speed Encoders (Interrupt Pins บนบอร์ด Arduino UNO R4) ---
const int ENCODER_L = 2; // ขา Interrupt ซ้าย
const int ENCODER_R = 3; // ขา Interrupt ขวา

volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }

void resetEncoders() {
  pulse_count_L = 0;
  pulse_count_R = 0;
}

// ============================================================================
// การตั้งค่าความเร็วคงที่ (CLOSED-LOOP SETTINGS)
// ไม่ใช้การฟิก PWM แต่กำหนดเป็น "ความเร็วเป้าหมาย: พัลส์ต่อวินาที"
// ============================================================================
// ล้อขนาด 6.5 ซม. จาน 20 รู -> 1 พัลส์ ≈ 1.021 ซม.
// 10.0 pulses/sec ≈ 10.2 ซม./วินาที (ความเร็วต่ำ นุ่มนวล ไม่สะบัด และเกาะถนนดีเยี่ยม)
float TARGET_SPEED_PPS = 10.0;       // ความเร็วเดินหน้า/ถอยหลัง (พัลส์/วินาที)
float TARGET_TURN_SPEED_PPS = 8.0;   // ความเร็วตอนหมุนรอบตัว (พัลส์/วินาที)

// พารามิเตอร์ของ PI Controller และการ Balance ล้อซ้าย-ขวา
const int PWM_BASE = 3000;           // PWM จุดเริ่มต้น (สเกล 12-bit: 0 - 4095)
const int PWM_MIN  = 2000;            // PWM ต่ำสุด
const int PWM_MAX  = 3800;           // PWM สูงสุด (เผื่อเหลือตอนแบตอ่อนมาก)

const float KP_SPEED = 40.0;         // ปรับแก้ตาม Error พัลส์ปัจจุบัน
const float KI_SPEED = 1.2;          // สะสม Error (ชดเชยแรงดันถ่านตก/แรงเสียดทาน)
const float KP_SYNC  = 10.0;         // ดึงล้อซ้าย-ขวาให้เท่ากัน รถวิ่งตรงเป๊ะ

// --- เรขาคณิตรถ สำหรับแปลง "องศาที่อยากหมุน" เป็น "จำนวนพัลส์" ---
const int DISK_SLOTS = 20;              // จำนวนรูบนแผ่น encoder
const float WHEEL_DIAMETER_CM = 6.5;    // เส้นผ่านศูนย์กลางล้อ (ซม.)
const float TRACK_WIDTH_CM = 14.5;      // ระยะห่างล้อซ้าย-ขวา (ซม.)
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;
float TURN_DEG_SCALE = 1.0;             // ตัวคูณชดเชยการหมุนจริง

unsigned long degToPulses(float target_deg) {
  return (unsigned long)(((target_deg * TURN_DEG_SCALE) / 360.0) * (3.14159265 * TRACK_WIDTH_CM / CM_PER_PULSE));
}

// --- ตำแหน่ง/ทิศทางรถ (Dead-reckoning) ---
float pos_x = 0.0;
float pos_y = 0.0;
float heading_deg = 0.0;

float normalizeAngle(float a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// --- ค่าคงที่แต่ละ leg ของเส้นทาง (ตามสเปกเดิมของ opy) ---
const float LEG1_FORWARD_CM = 360;
const float LEG1_SPIN_DEG = 625; const bool LEG1_SPIN_RIGHT = true;
const float LEG1_TURN_DEG = 0.0;  const bool LEG1_TURN_LEFT = true;
const float LEG1_CORRECTION_DEG = 30.0;
const float LEG1_CORRECTION_AT_CM = 200.0;

const float LEG2_FORWARD_CM = 40.0;
const float LEG2_SPIN_DEG = 540; const bool LEG2_SPIN_RIGHT = false;
const float LEG2_TURN_DEG = 0;    const bool LEG2_TURN_LEFT = false;
const float LEG2_CORRECTION_DEG = 20.0;
const float LEG2_CORRECTION_AT_CM = 50.0;
const float LEG2_CORRECTION2_DEG = -20.0;
const float LEG2_CORRECTION2_AT_CM = 300.0;

const float LEG3_FORWARD_CM = 370.0;
const float LEG3_SPIN_DEG = 895; const bool LEG3_SPIN_RIGHT = true;
const float LEG3_TURN_DEG = 0.0;  const bool LEG3_TURN_LEFT = true;
const float LEG3_CORRECTION_DEG = -15;
const float LEG3_CORRECTION_AT_CM = 10.0;
const float LEG3_CORRECTION2_DEG = -25.0;
const float LEG3_CORRECTION2_AT_CM = 200.0;

const float LEG4_FORWARD_CM = 460.0;
const float LEG4_SPIN_DEG = 645.0; const bool LEG4_SPIN_RIGHT = false;
const float LEG4_TURN_DEG = 0.0;    const bool LEG4_TURN_LEFT = false;
const float LEG4_CORRECTION_DEG = -20.0;
const float LEG4_CORRECTION_AT_CM = 10.0;
const float LEG4_CORRECTION2_DEG = 20.0;
const float LEG4_CORRECTION2_AT_CM = 250.0;

// พิมพ์ค่าออก Serial Monitor เพื่อวิเคราะห์การทำงานแบบ Real-time
void printTelemetry(float target_p, int pwmL, int pwmR) {
  static unsigned long last_print = 0;
  if (millis() - last_print >= 250) {
    Serial.print("Target: "); Serial.print((int)target_p);
    Serial.print(" | L: "); Serial.print(pulse_count_L);
    Serial.print(" (PWM:"); Serial.print(pwmL); Serial.print(")");
    Serial.print(" | R: "); Serial.print(pulse_count_R);
    Serial.print(" (PWM:"); Serial.print(pwmR); Serial.println(")");
    last_print = millis();
  }
}

// ============================================================================
// ฟังก์ชันหัวใจหลัก: ควบคุมความเร็วแบบ Closed-Loop
// ============================================================================
void runClosedLoopTrajectory(unsigned long target_pulses, float target_pps, bool is_turn) {
  unsigned long start_time = millis();
  
  // คำนวณ Safety Timeout อัตโนมัติจากระยะทางและความเร็ว (บวกเผื่อ 6 วินาที)
  unsigned long expected_duration_ms = (unsigned long)((target_pulses / target_pps) * 1000.0f);
  unsigned long timeout_ms = expected_duration_ms + 6000;

  float integral_L = 0.0f;
  float integral_R = 0.0f;
  unsigned long last_loop_time = start_time;

  while (true) {
    unsigned long now = millis();
    if (now - start_time > timeout_ms) {
      Serial.println("[WARN] Safety Timeout Reached!");
      break;
    }

    // หยุดเมื่อพัลส์เฉลี่ยถึงเป้าหมาย
    unsigned long avg_pulses = (pulse_count_L + pulse_count_R) / 2;
    if (avg_pulses >= target_pulses) {
      break;
    }

    float dt = (now - last_loop_time) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last_loop_time = now;

    // คำนวณพัลส์ที่ควรจะได้ ณ เวลาปัจจุบัน (Trajectory Generator)
    float elapsed_sec = (now - start_time) / 1000.0f;
    float desired_pulses = target_pps * elapsed_sec;
    if (desired_pulses > (float)target_pulses) desired_pulses = (float)target_pulses;

    // คำนวณ Error ระหว่างเป้าหมายกับพัลส์จริงของแต่ละล้อ
    float err_L = desired_pulses - (float)pulse_count_L;
    float err_R = desired_pulses - (float)pulse_count_R;

    // สะสม Error (Integral) เพื่อชดเชยแรงดันแบตเตอรี่ตก
    integral_L += err_L * dt * 50.0f;
    integral_R += err_R * dt * 50.0f;
    integral_L = constrain(integral_L, -500.0f, 2000.0f); // Anti-windup
    integral_R = constrain(integral_R, -500.0f, 2000.0f);

    // คำนวณส่วนต่างระหว่างสองล้อ เพื่อดึงให้รถวิ่งตรงเป๊ะ (Wheel Sync)
    long sync_diff = (long)pulse_count_L - (long)pulse_count_R; // ซ้ายวิ่งเกิน = บวก
    float sync_correction = (float)sync_diff * KP_SYNC;

    // คำนวณค่า PWM สุทธิ
    int pwmL = (int)(PWM_BASE + (err_L * KP_SPEED) + (integral_L * KI_SPEED) - sync_correction);
    int pwmR = (int)(PWM_BASE + (err_R * KP_SPEED) + (integral_R * KI_SPEED) + sync_correction);

    pwmL = constrain(pwmL, PWM_MIN, PWM_MAX);
    pwmR = constrain(pwmR, PWM_MIN, PWM_MAX);

    analogWrite(ENA, pwmL);
    analogWrite(ENB, pwmR);

    printTelemetry(desired_pulses, pwmL, pwmR);
    delay(15); // ควบคุมรอบลูปประมาณ 50-60 Hz
  }

  // เบรกหยุดทันทีหลังเสร็จสิ้นระยะ
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ----------------------------------------------------------------------------
// ฟังก์ชันเคลื่อนที่ต่างๆ
// ----------------------------------------------------------------------------
void stopRobot(int time_ms) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(time_ms);
}

void forward(float distance_cm) {
  resetEncoders();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  unsigned long target_pulses = (unsigned long)(distance_cm / CM_PER_PULSE);
  Serial.print("\n>>> FORWARD: "); Serial.print(distance_cm);
  Serial.print(" cm ("); Serial.print(target_pulses); Serial.println(" pulses)");

  runClosedLoopTrajectory(target_pulses, TARGET_SPEED_PPS, false);

  float rad = heading_deg * PI / 180.0;
  pos_x += distance_cm * cos(rad);
  pos_y += distance_cm * sin(rad);
}

void backward(float distance_cm) {
  resetEncoders();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  unsigned long target_pulses = (unsigned long)(distance_cm / CM_PER_PULSE);
  Serial.print("\n>>> BACKWARD: "); Serial.print(distance_cm);
  Serial.print(" cm ("); Serial.print(target_pulses); Serial.println(" pulses)");

  runClosedLoopTrajectory(target_pulses, TARGET_SPEED_PPS, false);

  float rad = heading_deg * PI / 180.0;
  pos_x -= distance_cm * cos(rad);
  pos_y -= distance_cm * sin(rad);
}

void around_left(float target_deg) {
  resetEncoders();
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // ล้อซ้ายถอยหลัง
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // ล้อขวาเดินหน้า

  unsigned long target_pulses = degToPulses(target_deg);
  Serial.print("\n>>> AROUND LEFT: "); Serial.print(target_deg);
  Serial.print(" deg ("); Serial.print(target_pulses); Serial.println(" pulses)");

  runClosedLoopTrajectory(target_pulses, TARGET_TURN_SPEED_PPS, true);
  heading_deg = normalizeAngle(heading_deg + target_deg);
}

void around_right(float target_deg) {
  resetEncoders();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  // ล้อซ้ายเดินหน้า
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // ล้อขวาถอยหลัง

  unsigned long target_pulses = degToPulses(target_deg);
  Serial.print("\n>>> AROUND RIGHT: "); Serial.print(target_deg);
  Serial.print(" deg ("); Serial.print(target_pulses); Serial.println(" pulses)");

  runClosedLoopTrajectory(target_pulses, TARGET_TURN_SPEED_PPS, true);
  heading_deg = normalizeAngle(heading_deg - target_deg);
}

void runLeg(float forward_cm,
            float spin_deg, bool spin_right,
            float turn_deg, bool turn_left,
            float corr1_deg, float corr1_at_cm,
            float corr2_deg, float corr2_at_cm) {
  float bp_cm[2];
  float bp_deg[2];
  int n = 0;
  if (corr1_at_cm > 0) { bp_cm[n] = corr1_at_cm; bp_deg[n] = corr1_deg; n++; }
  if (corr2_at_cm > 0) { bp_cm[n] = corr2_at_cm; bp_deg[n] = corr2_deg; n++; }
  if (n == 2 && bp_cm[0] > bp_cm[1]) {
    float tc = bp_cm[0]; bp_cm[0] = bp_cm[1]; bp_cm[1] = tc;
    float td = bp_deg[0]; bp_deg[0] = bp_deg[1]; bp_deg[1] = td;
  }

  float dist_so_far = 0;
  for (int i = 0; i < n; i++) {
    forward(bp_cm[i] - dist_so_far);
    dist_so_far = bp_cm[i];
    stopRobot(1000); // หยุดนิ่ง 1 วินาทีก่อนหมุนแก้เอียง
    if (bp_deg[i] > 0) around_left(bp_deg[i]);
    else if (bp_deg[i] < 0) around_right(-bp_deg[i]);
  }
  forward(forward_cm - dist_so_far);

  if (spin_deg != 0 || turn_deg != 0) {
    stopRobot(1000); // หยุดนิ่ง 1 วินาทีก่อนหมุนปิดท้าย leg
  }
  if (spin_deg != 0) {
    if (spin_right) around_right(spin_deg); else around_left(spin_deg);
  }
  if (turn_deg != 0) {
    if (turn_left) around_left(turn_deg); else around_right(turn_deg);
  }
}

// ----------------------------------------------------------------------------
// SETUP & LOOP
// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);

  analogWriteResolution(12); // กำหนดความละเอียด PWM 0 - 4095 สำหรับ Arduino UNO R4

  Serial.println("==================================================");
  Serial.println("     OPY CLOSED-LOOP AUTO SPEED CONTROLLER        ");
  Serial.println("==================================================");
  Serial.print("Target Speed : "); Serial.print(TARGET_SPEED_PPS); Serial.println(" pulses/s (~10 cm/s)");
  Serial.print("Turn Speed   : "); Serial.print(TARGET_TURN_SPEED_PPS); Serial.println(" pulses/s");
  Serial.println("Waiting 2 seconds before start...");

  stopRobot(2000); // รอ 2 วินาทีก่อนเริ่มวิ่ง

  // เริ่มวิ่ง 4 Leg ตามเส้นทางเดิม
  Serial.println("\n--- RUNNING LEG 1 ---");
  runLeg(LEG1_FORWARD_CM, LEG1_SPIN_DEG, LEG1_SPIN_RIGHT, LEG1_TURN_DEG, LEG1_TURN_LEFT,
         LEG1_CORRECTION_DEG, LEG1_CORRECTION_AT_CM, 0, 0);
  stopRobot(500);

  Serial.println("\n--- RUNNING LEG 2 ---");
  runLeg(LEG2_FORWARD_CM, LEG2_SPIN_DEG, LEG2_SPIN_RIGHT, LEG2_TURN_DEG, LEG2_TURN_LEFT,
         LEG2_CORRECTION_DEG, LEG2_CORRECTION_AT_CM, LEG2_CORRECTION2_DEG, LEG2_CORRECTION2_AT_CM);
  stopRobot(500);

  Serial.println("\n--- RUNNING LEG 3 ---");
  runLeg(LEG3_FORWARD_CM, LEG3_SPIN_DEG, LEG3_SPIN_RIGHT, LEG3_TURN_DEG, LEG3_TURN_LEFT,
         LEG3_CORRECTION_DEG, LEG3_CORRECTION_AT_CM, LEG3_CORRECTION2_DEG, LEG3_CORRECTION2_AT_CM);
  stopRobot(500);

  Serial.println("\n--- RUNNING LEG 4 ---");
  runLeg(LEG4_FORWARD_CM, LEG4_SPIN_DEG, LEG4_SPIN_RIGHT, LEG4_TURN_DEG, LEG4_TURN_LEFT,
         LEG4_CORRECTION_DEG, LEG4_CORRECTION_AT_CM, LEG4_CORRECTION2_DEG, LEG4_CORRECTION2_AT_CM);
  stopRobot(500);

  Serial.println("\n=== ALL LEGS COMPLETED ===");
}

void loop() {
  // จบการทำงาน ไม่วนซ้ำ
}
