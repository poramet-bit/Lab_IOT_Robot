// ============================================================================
// OPY MAX4 — LM393 closed-loop odometry (ไฟล์แยก คนละสเก็ตช์กับตัวอื่น)
// ----------------------------------------------------------------------------
// โครงไฟล์ (ไล่จากบนลงล่าง):
//   1) CONFIG      ค่าที่ปรับบ่อย: ขา, ความเร็ว, เกน, เรขาคณิตรถ
//   2) ROUTE       ค่าเส้นทาง LEG1-LEG3 แบบค่าคงที่ LEGx_* (เหมือนไฟล์ต้นฉบับ)
//   3) ENCODER     อ่านพัลส์ LM393 -> รอบล้อ
//   4) ODOMETRY    พัลส์ -> ระยะทาง -> มุมองศารถ (heading) -> ตำแหน่ง x,y
//   5) MOTOR       สั่งทิศ/กำลังล้อ + เบรก (ชั้นล่างสุด ที่เดียวที่แตะขา IN/EN)
//   6) STOP/LOCK   เบรกนิ่ง 1 วิ และล็อกตำแหน่งหลังหมุน
//   7) MOTION      วิ่งตรง / หมุนอยู่กับที่ / ไปยังพิกัด
//   8) ROUTE RUN   runLeg0 + runLeg + setup/loop
// ----------------------------------------------------------------------------
// ไฟล์นี้ต่างจากตัวเดิม:
//   - ล้อหมุนเต็มกำลังตลอด: PWM 4095, ปิด ramp/ชะลอ, trim = 0 (ค่าเดิมคอมเมนต์ไว้ทุกตัว)
//   - ค่าเส้นทาง LEG0-LEG3 ตั้งเป็น 0 ทั้งหมด ค่าเดิมคอมเมนต์ไว้ท้ายบรรทัด
//   - LEG 0 = หมุนตั้งทิศที่จุดเริ่มต้นก่อน แล้วค่อยวิ่ง LEG 1
// พฤติกรรมหลัก:
//   - ก่อนหมุน และเมื่อวิ่งครบระยะ: เบรกแล้วนิ่ง 1 วินาที (BRAKE_MS)
//   - หมุนเสร็จ: ล็อกตำแหน่ง (ดันกลับถ้าไถล) แล้วนิ่ง 1 วินาทีก่อนออกตัว (LOCK_MS)
//   - วิ่งตรง: PI บาลานซ์ล้อซ้าย-ขวา + ดึงกลับหา heading + ramp ออกตัว/ชะลอก่อนถึงเป้า
//   - สายมอเตอร์ต่อกลับขั้ว แก้ด้วยธง INVERT_L / INVERT_R ไม่ต้องสลับสายจริง
// ============================================================================

// ============================================================================
// 1) CONFIG
// ============================================================================

// --- ขามอเตอร์ ---
const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;

const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;

// --- ขา encoder LM393 ---
const int ENCODER_L = 3;  // ล้อซ้าย (code L)
const int ENCODER_R = 11; // ล้อขวา (code R)
// ถ้าค่า R ใน Serial ค้างที่ 0 แปลว่าขา 11 ไม่รับ interrupt -> ย้ายไปขา 2

// --- ทิศการหมุนของมอเตอร์ ---
// รถวิ่งถอยหลังทั้งที่สั่งเดินหน้า = สายมอเตอร์ต่อกลับขั้ว แก้ที่นี่ที่เดียว
// กลับทั้งสองล้อ = ทิศหมุนซ้าย/ขวาถูกแก้ให้ตรงตามจริงไปด้วย
// ถ้าเดินหน้าถูกแล้วแต่หมุนผิดข้าง = ล้อเดียวที่ต่อกลับ ให้กลับค่าเพียงตัวเดียว
const bool INVERT_L = true;
const bool INVERT_R = true;

// --- ความเร็ว (สเกล PWM 12-bit: 0-4095) --- ปรับเป็นสูงสุดแล้ว
const int SPEED = 4095;      // เดินหน้า/ถอยหลัง   // ค่าเดิม: 2900 (กลางค่อนสูง), ก่อนหน้านั้น: 4000
const int TURN_SPEED = 4095; // หมุนอยู่กับที่     // ค่าเดิม: 2500, ก่อนหน้านั้น: 4000

// --- ramp ออกตัว / ชะลอก่อนถึงเป้า ---
// ปิดทั้งคู่แล้ว เพื่อให้ล้อหมุนเต็มกำลังตลอดช่วงวิ่ง (ตามที่สั่ง)
// RAMP_UP_MS = 0 คือไม่ไต่ความเร็ว กระชากเต็มกำลังทันที
// DECEL_PULSES = 0 คือไม่ชะลอก่อนถึงเป้า เบรกเต็มแรงตอนถึงพอดี
// อยากได้ความแม่นระยะ/มุมคืน ให้ใส่ค่าที่คอมเมนต์ไว้กลับไป
const int START_PWM = 4095;             // ค่าเดิม: 2200 (ก่อนหน้านั้น 1800)
const int FINISH_PWM = 4095;            // ค่าเดิม: 1900 (ก่อนหน้านั้น 1700)
const unsigned long RAMP_UP_MS = 0;     // ค่าเดิม: 400 (ก่อนหน้านั้น 350)
const unsigned long DECEL_PULSES = 0;   // ค่าเดิม: 10 (ก่อนหน้านั้น 6)

// --- trim แก้ล้อวิ่งไม่เท่ากัน (ใช้เฉพาะตอนวิ่งตรง) ---
// ตั้ง 0 ทั้งคู่ เพราะ trim คือการหักกำลังล้อออก จะทำให้ไม่ได้ความเร็วสูงสุด
// ถ้ารถเบี้ยวมากจนคุมไม่อยู่ ให้ใส่ B_L กลับ (ค่าเดิม 250) แล้วยอมเสียความเร็วเล็กน้อย
const int B_L = 0;  // ค่าเดิม: 250 (ตอน SPEED=4000) / 180 (ตอน SPEED=2900)
const int B_R = 0;

// --- เกนคุมทิศตอนวิ่งตรง ---
const float KP_BALANCE = 5.0;        // ตามผลต่างพัลส์สด ซ้าย-ขวา (สูง = แก้ไว แต่ส่ายง่าย)
const float KI_BALANCE = 0.15;       // สะสมผลต่างพัลส์ กันเอียงค้างทางเดียวตลอดเส้น
const float KP_HEADING = 12.0;       // ดึงกลับหา heading ตอนเริ่มวิ่ง (PWM ต่อ 1 องศาเบี้ยว)
const float BALANCE_I_LIMIT = 400.0; // เพดาน integral กัน windup

// --- เวลาเบรก / ล็อกตำแหน่ง ---
const int BRAKE_MS = 1000;                // เบรกนิ่ง 1 วิ ตอนสุดระยะและก่อนหมุน
const int LOCK_MS = 1000;                 // ล็อกตำแหน่งหลังหมุน นิ่ง 1 วิ ก่อนออกตัว
const unsigned long LOCK_TOL_PULSES = 1;  // ไถลเกินกี่พัลส์จึงดันกลับ
const int LOCK_NUDGE_PWM = 2200;          // แรงดันกลับตอนล็อกตำแหน่ง
const int LOCK_NUDGE_MS = 40;             // เวลาดันกลับต่อครั้ง

// --- เรขาคณิตรถ (ต้องวัดจากรถจริง) ---
const int DISK_SLOTS = 20;              // จำนวนรูบนแผ่น encoder (1 รอบล้อ)
const float WHEEL_DIAMETER_CM = 6.5;    // เส้นผ่านศูนย์กลางล้อ
const float TRACK_WIDTH_CM = 14.5;      // ระยะห่างล้อซ้าย-ขวา
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE_THEORY = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;

// ตัวชดเชยจากการวัดจริง (ล้อลื่นไถล/เฟืองสูญเสีย)
// คาลิเบรตระยะ: สั่งวิ่ง 100cm วัดจริง -> เกิน = เพิ่มค่านี้, ขาด = ลดค่านี้
const float CM_PER_PULSE_CAL = 1.0;
const float CM_PER_PULSE = CM_PER_PULSE_THEORY * CM_PER_PULSE_CAL;
const float TURN_DEG_SCALE = 1.0;       // ชดเชยมุมหมุนจริง เริ่ม 1.0 แล้วทดสอบค่อยปรับ

// --- safety timeout กันค้างถ้าพัลส์ขาด/ล้อไม่หมุน ---
// ต้องมากกว่าเวลาจริงต่อพัลส์ที่ความเร็วช้าสุด ไม่งั้น timeout ตัดก่อนถึงเป้าใน leg ยาว
const float TIMEOUT_MS_PER_PULSE = 14.0; // เผื่อไว้กว้าง (ค่านี้เป็นเพดาน ไม่ได้บังคับความเร็ว)
const unsigned long MIN_TIMEOUT_MS = 3000;

// --- โหมดทิศล้อ บอก odometry ว่าพัลส์ที่นับได้เดินไปทางไหน (LM393 ไม่รู้ทิศเอง) ---
const int MODE_FORWARD = 0;
const int MODE_BACKWARD = 1;
const int MODE_SPIN_LEFT = 2;
const int MODE_SPIN_RIGHT = 3;

// ============================================================================
// 2) ROUTE — ค่าเส้นทาง แก้ที่นี่ที่เดียว (LEG 0 = หมุนตั้งทิศที่จุดเริ่มต้น)
// ----------------------------------------------------------------------------
// ชื่อค่าแต่ละตัวหมายถึงอะไร:
//   LEGx_FORWARD_CM        = ระยะเดินหน้ารวมของ leg นั้น (ซม.)
//   LEGx_SPIN_DEG          = องศาที่หมุนปิดท้าย leg (0 = ไม่หมุน)
//   LEGx_SPIN_RIGHT        = true หมุนขวา / false หมุนซ้าย
//   LEGx_TURN_DEG          = องศาหมุนปรับทิศเพิ่มอีกชั้นหลังสปิน (0 = ไม่ใช้)
//   LEGx_TURN_LEFT         = true หมุนซ้าย / false หมุนขวา
//   LEGx_CORRECTION_AT_CM  = จุดที่ 1: วิ่งครบกี่ ซม. จึงหยุดหักแก้เอียง (0 = ปิดจุดนี้)
//   LEGx_CORRECTION_DEG    = จุดที่ 1: หักกี่องศา (+ ซ้าย / - ขวา)
//   LEGx_CORRECTION2_*     = จุดที่ 2 (ระยะ + องศา) เหมือนจุดที่ 1
//   LEGx_CORRECTION3_*     = จุดที่ 3 (ระยะ + องศา) เหมือนจุดที่ 1
// ค่าทั้งหมดตั้งเป็น 0 ตามที่สั่ง -> รถยังไม่วิ่งจนกว่าจะเติมค่า ค่าเดิมคอมเมนต์ไว้ท้ายบรรทัด
// ============================================================================
// 7.1-7.2V //150 กลับหลัง
// --- LEG 0: หมุนตั้งทิศที่จุดเริ่มต้น (จุด 0) ก่อนออกวิ่ง LEG 1 ---
// ไม่มีระยะเดินหน้า มีแต่การหมุนอยู่กับที่
// LEG0_SPIN_DEG = 0 คือข้าม LEG 0 ไปเริ่ม LEG 1 เลย
const float LEG0_SPIN_DEG = 44.5;    const bool LEG0_SPIN_RIGHT = false;  // true หมุนขวา / false หมุนซ้าย
const float LEG0_TURN_DEG = 0;    const bool LEG0_TURN_LEFT = true;   // หมุนปรับทิศเพิ่มอีกชั้น (0 = ไม่ใช้)

// --- LEG 1 ---
const float LEG1_FORWARD_CM = 160;                            // ค่าเดิม: 0 (เคยใช้ 320 / 820)
const float LEG1_SPIN_DEG = 44.5;    const bool LEG1_SPIN_RIGHT = true;  // ค่าเดิม: 89.5 หมุนขวา (ชุดแรกสุด: 563.75 ขวา)
const float LEG1_TURN_DEG = 0;    const bool LEG1_TURN_LEFT = true;   // ค่าเดิม: 0
const float LEG1_CORRECTION_AT_CM = 0;                        // ค่าเดิม: 0 (ชุดแรกสุด: 200)
const float LEG1_CORRECTION_DEG = 0;                          // ค่าเดิม: 0 (ชุดแรกสุด: +20)
const float LEG1_CORRECTION2_AT_CM = 0;                       // ค่าเดิม: 0
const float LEG1_CORRECTION2_DEG = 0;                         // ค่าเดิม: 0
const float LEG1_CORRECTION3_AT_CM = 0;                       // ค่าเดิม: 0
const float LEG1_CORRECTION3_DEG = 0;                         // ค่าเดิม: 0

// --- LEG 2 ---
const float LEG2_FORWARD_CM = 500;                              // ค่าเดิม: 400 (ชุดแรกสุด: 1150)
const float LEG2_SPIN_DEG = 0;    const bool LEG2_SPIN_RIGHT = false; // ค่าเดิม: 0 (ชุดแรกสุด: 943.75 ซ้าย)
const float LEG2_TURN_DEG = 0;    const bool LEG2_TURN_LEFT = false;  // ค่าเดิม: 0
const float LEG2_CORRECTION_AT_CM = 0;                        // ค่าเดิม: 0 (ชุดแรกสุด: 100)
const float LEG2_CORRECTION_DEG = 0;                          // ค่าเดิม: 0 (ชุดแรกสุด: +10)
const float LEG2_CORRECTION2_AT_CM = 0;                       // ค่าเดิม: 0 (ชุดแรกสุด: 600)
const float LEG2_CORRECTION2_DEG = 0;                         // ค่าเดิม: 0 (ชุดแรกสุด: +30)
const float LEG2_CORRECTION3_AT_CM = 0;                       // ค่าเดิม: 0 (ชุดแรกสุด: 900)
const float LEG2_CORRECTION3_DEG = 0;                         // ค่าเดิม: 0 (ชุดแรกสุด: +10)

// --- LEG 3 ---
const float LEG3_FORWARD_CM = 0;                              // ค่าเดิม: 860
const float LEG3_SPIN_DEG = 360;    const bool LEG3_SPIN_RIGHT = true;  // ค่าเดิม: 565 หมุนขวา
const float LEG3_TURN_DEG = 0;    const bool LEG3_TURN_LEFT = true;   // ค่าเดิม: 0
const float LEG3_CORRECTION_AT_CM = 0;                        // ค่าเดิม: 50
const float LEG3_CORRECTION_DEG = 0;                          // ค่าเดิม: +35
const float LEG3_CORRECTION2_AT_CM = 0;                       // ค่าเดิม: 400
const float LEG3_CORRECTION2_DEG = 0;                         // ค่าเดิม: -10
const float LEG3_CORRECTION3_AT_CM = 0;                       // ค่าเดิม: 0
const float LEG3_CORRECTION3_DEG = 0;                         // ค่าเดิม: 0

const int MAX_CORRECTIONS = 3;

// ============================================================================
// 3) ENCODER — LM393
// ============================================================================

volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

unsigned long total_pulses_L = 0; // สะสมตลอดเส้นทาง ใช้รายงานรอบล้อรวม
unsigned long total_pulses_R = 0;

void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }

void encoderSetup() {
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);
}

void readEncoders(unsigned long *l, unsigned long *r) {
  noInterrupts();
  *l = pulse_count_L;
  *r = pulse_count_R;
  interrupts();
}

// รอบล้อสะสม (1 รอบ = DISK_SLOTS พัลส์)
float wheelRevsL() { return (total_pulses_L + pulse_count_L) / (float)DISK_SLOTS; }
float wheelRevsR() { return (total_pulses_R + pulse_count_R) / (float)DISK_SLOTS; }

unsigned long cmToPulses(float distance_cm) {
  return (unsigned long)(distance_cm / CM_PER_PULSE);
}

// องศาที่อยากหมุน -> จำนวนพัลส์ (arc length ของล้อที่หมุนรอบจุดกึ่งกลางตัวถัง)
unsigned long degToPulses(float target_deg) {
  return (unsigned long)(((target_deg * TURN_DEG_SCALE) / 360.0) *
                         (3.14159265 * TRACK_WIDTH_CM / CM_PER_PULSE));
}

unsigned long computeTimeoutMs(unsigned long target_pulses) {
  unsigned long t = (unsigned long)(target_pulses * TIMEOUT_MS_PER_PULSE);
  return t < MIN_TIMEOUT_MS ? MIN_TIMEOUT_MS : t;
}

// ============================================================================
// 4) ODOMETRY — พัลส์ -> มุมองศารถ + ตำแหน่ง
// origin (0,0) = จุดเริ่มต้นตอนเปิดเครื่อง, heading 0 = ทิศที่รถหันตอนเริ่ม
// ============================================================================

float pos_x = 0.0;
float pos_y = 0.0;
float heading_deg = 0.0;  // มุมองศาของตัวรถ คำนวณสดทุกลูป ทั้งตอนวิ่งตรงและตอนหมุน

unsigned long odo_prev_L = 0; // พัลส์ครั้งก่อน ใช้คำนวณแบบเพิ่มทีละก้าว
unsigned long odo_prev_R = 0;

float normalizeAngle(float a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// เริ่มนับพัลส์รอบใหม่ (ย้ายยอดเดิมไปกองสะสม แล้วเคลียร์)
void resetEncoders() {
  noInterrupts();
  total_pulses_L += pulse_count_L;
  total_pulses_R += pulse_count_R;
  pulse_count_L = 0;
  pulse_count_R = 0;
  interrupts();
  odo_prev_L = 0;
  odo_prev_R = 0;
}

void updateOdometry(unsigned long pl, unsigned long pr, int mode) {
  long dl = (long)pl - (long)odo_prev_L;
  long dr = (long)pr - (long)odo_prev_R;
  odo_prev_L = pl;
  odo_prev_R = pr;
  if (dl == 0 && dr == 0) return;

  float sl = dl * CM_PER_PULSE;
  float sr = dr * CM_PER_PULSE;

  // ใส่เครื่องหมายตามทิศล้อจริง เพราะ encoder นับขึ้นอย่างเดียว
  if (mode == MODE_BACKWARD)        { sl = -sl; sr = -sr; }
  else if (mode == MODE_SPIN_LEFT)  { sl = -sl; }  // ล้อซ้ายถอย ล้อขวาเดินหน้า
  else if (mode == MODE_SPIN_RIGHT) { sr = -sr; }  // ล้อซ้ายเดินหน้า ล้อขวาถอย

  float ds = (sl + sr) / 2.0;
  float dth_deg = ((sr - sl) / TRACK_WIDTH_CM) * (180.0 / PI) * TURN_DEG_SCALE;

  float mid_rad = (heading_deg + dth_deg / 2.0) * PI / 180.0;
  pos_x += ds * cos(mid_rad);
  pos_y += ds * sin(mid_rad);
  heading_deg = normalizeAngle(heading_deg + dth_deg);
}

// พิมพ์สถานะทุก 200ms: พัลส์ / รอบล้อ / มุมองศารถ / ตำแหน่ง
void printStatus(unsigned long pl, unsigned long pr) {
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time < 200) return;
  last_print_time = millis();

  Serial.print("L:");        Serial.print(pl);
  Serial.print(" R:");       Serial.print(pr);
  Serial.print(" | rev L:"); Serial.print(wheelRevsL(), 2);
  Serial.print(" R:");       Serial.print(wheelRevsR(), 2);
  Serial.print(" | deg:");   Serial.print(heading_deg, 1);
  Serial.print(" | x:");     Serial.print(pos_x, 1);
  Serial.print(" y:");       Serial.println(pos_y, 1);
}

// ============================================================================
// 5) MOTOR — ชั้นล่างสุด ที่เดียวที่แตะขา IN/EN
// ============================================================================

// dir: 1 = เดินหน้า, -1 = ถอยหลัง, 0 = เบรก (ผ่านธง INVERT_L)
void setMotorL(int dir) {
  int d = INVERT_L ? -dir : dir;
  if (d > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (d < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else            { digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); }
}

// dir: 1 = เดินหน้า, -1 = ถอยหลัง, 0 = เบรก (ผ่านธง INVERT_R)
void setMotorR(int dir) {
  int d = INVERT_R ? -dir : dir;
  if (d > 0)      { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (d < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else            { digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); }
}

void applyPower(int pwm_l, int pwm_r) {
  analogWrite(ENA, constrain(pwm_l, 0, 4095));
  analogWrite(ENB, constrain(pwm_r, 0, 4095));
}

// เบรกจริง (short brake: ขา IN ทั้งคู่เท่ากัน + EN เต็ม) หยุดเฉียบกว่าปล่อยไหล
void brakeMotors() {
  setMotorL(0);
  setMotorR(0);
  applyPower(4095, 4095);
}

// ตัดไฟมอเตอร์ ปล่อยอิสระ ใช้ตอนจบงานเท่านั้น
void motorsOff() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  applyPower(0, 0);
}

void motorSetup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWriteResolution(12); // PWM 0-4095 (Arduino UNO R4 / ESP32)
}

// ============================================================================
// 6) STOP / LOCK
// ============================================================================

// เบรกแล้วนิ่งตามเวลาที่กำหนด (ใช้ก่อนหมุน และเมื่อวิ่งครบระยะ)
void brakeHold(int time_ms) {
  brakeMotors();
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)time_ms) {
    unsigned long pl, pr;
    readEncoders(&pl, &pr);
    printStatus(pl, pr);
    delay(5);
  }
}

// เข้ากันได้กับโค้ดเดิม: หยุด = เบรกค้าง
void stopRobot(int time_ms) {
  brakeHold(time_ms);
}

// ดันสวนสั้น ๆ ดึงมุมกลับ แล้วเบรกค้างต่อ (ใช้ภายใน lockPosition)
void nudgeBack(int spin_mode) {
  if (spin_mode == MODE_SPIN_LEFT) { setMotorL(1);  setMotorR(-1); } // เพิ่งหมุนซ้าย -> ดันขวา
  else                             { setMotorL(-1); setMotorR(1);  } // เพิ่งหมุนขวา -> ดันซ้าย
  applyPower(LOCK_NUDGE_PWM, LOCK_NUDGE_PWM);
  delay(LOCK_NUDGE_MS);
  brakeMotors();
}

// ล็อกตำแหน่งหลังหมุนเสร็จ: เบรกค้าง จับพัลส์ที่ไถลต่อ แล้วดันกลับให้มุมคงเดิม
void lockPosition(int time_ms, int spin_mode) {
  brakeMotors();

  unsigned long pl, pr;
  readEncoders(&pl, &pr);
  updateOdometry(pl, pr, spin_mode);
  unsigned long base_l = pl;
  unsigned long base_r = pr;

  int back_mode = (spin_mode == MODE_SPIN_LEFT) ? MODE_SPIN_RIGHT : MODE_SPIN_LEFT;
  unsigned long t0 = millis();

  while (millis() - t0 < (unsigned long)time_ms) {
    readEncoders(&pl, &pr);
    updateOdometry(pl, pr, spin_mode);

    unsigned long drift = ((pl - base_l) + (pr - base_r)) / 2;
    if (drift > LOCK_TOL_PULSES) {
      nudgeBack(spin_mode);
      readEncoders(&pl, &pr);
      updateOdometry(pl, pr, back_mode); // พัลส์ช่วงดันกลับนับเป็นทิศตรงข้าม
      base_l = pl;
      base_r = pr;
    }

    printStatus(pl, pr);
    delay(5);
  }
}

// ============================================================================
// 7) MOTION
// ============================================================================

// กำลังขับ ณ ขณะนั้น: ไต่ขึ้นตอนออกตัว และชะลอลงก่อนถึงเป้า
int rampPower(int cruise, unsigned long elapsed_ms,
              unsigned long done_pulses, unsigned long target_pulses) {
  int p = cruise;

  // ไต่ขึ้นตอนออกตัว (ข้ามเมื่อ RAMP_UP_MS = 0)
  if (RAMP_UP_MS > 0 && elapsed_ms < RAMP_UP_MS) {
    p = START_PWM + (int)((long)(cruise - START_PWM) * (long)elapsed_ms / (long)RAMP_UP_MS);
  }

  // ชะลอลงก่อนถึงเป้า (ข้ามเมื่อ DECEL_PULSES = 0)
  if (DECEL_PULSES > 0) {
    unsigned long remaining = (target_pulses > done_pulses) ? (target_pulses - done_pulses) : 0;
    if (remaining <= DECEL_PULSES) {
      int q = FINISH_PWM + (int)((long)(cruise - FINISH_PWM) * (long)remaining / (long)DECEL_PULSES);
      if (q < p) p = q;
    }
  }
  return p;
}

// ค่าหักเลี้ยวตอนวิ่งตรง: ผลต่างพัลส์ (P + I) + มุมที่เบี้ยวจาก heading เป้าหมาย
// ผลลัพธ์บวก = ล้อซ้ายเกิน ต้องลดซ้าย/เพิ่มขวา
int straightCorrection(unsigned long pl, unsigned long pr,
                       float heading_ref, bool reverse, float *i_term) {
  long err = (long)pl - (long)pr;
  *i_term = constrain(*i_term + err * 0.01, -BALANCE_I_LIMIT, BALANCE_I_LIMIT);

  float head_err = normalizeAngle(heading_deg - heading_ref); // เบี้ยวซ้าย = บวก
  if (reverse) head_err = -head_err;

  return (int)(err * KP_BALANCE + (*i_term) * KI_BALANCE + head_err * KP_HEADING);
}

// วิ่งตรงแบบ closed-loop แล้วเบรกนิ่ง 1 วิ เมื่อถึงระยะที่กำหนด
void driveStraight(float distance_cm, bool reverse) {
  if (distance_cm <= 0) return;

  resetEncoders();
  int dir = reverse ? -1 : 1;
  setMotorL(dir);
  setMotorR(dir);

  int mode = reverse ? MODE_BACKWARD : MODE_FORWARD;
  unsigned long target_pulses = cmToPulses(distance_cm);
  unsigned long timeout_ms = computeTimeoutMs(target_pulses);
  float heading_ref = heading_deg;  // มุมที่ต้องรักษาตลอดช่วงวิ่งตรง
  float i_term = 0.0;
  unsigned long start_time = millis();

  unsigned long pl = 0, pr = 0;
  while (((pl + pr) / 2) < target_pulses) {
    if (millis() - start_time > timeout_ms) break; // safety timeout

    readEncoders(&pl, &pr);
    updateOdometry(pl, pr, mode);

    int correction = straightCorrection(pl, pr, heading_ref, reverse, &i_term);
    int base = rampPower(SPEED, millis() - start_time, (pl + pr) / 2, target_pulses);
    applyPower(base - B_L - correction, base - B_R + correction);

    printStatus(pl, pr);
    delay(5);
  }

  readEncoders(&pl, &pr);
  updateOdometry(pl, pr, mode);
  brakeHold(BRAKE_MS); // ถึงระยะที่กำหนด -> เบรกแล้วนิ่ง BRAKE_MS
}

void forward(float distance_cm)  { driveStraight(distance_cm, false); }
void backward(float distance_cm) { driveStraight(distance_cm, true); }

// หมุนอยู่กับที่: เบรกสั้น ๆ ก่อนหมุน -> หมุนโดยบังคับสองล้อให้เท่ากัน -> ล็อกตำแหน่ง 1 วิ
// ไม่ใส่ trim B_L/B_R เพื่อไม่ให้เอียงไปทางใดทางหนึ่ง
void spinInPlace(float target_deg, bool left) {
  if (target_deg <= 0) return;

  brakeHold(BRAKE_MS); // ก่อนหมุน: เบรกแล้วนิ่ง BRAKE_MS
  resetEncoders();

  if (left) { setMotorL(-1); setMotorR(1); }  // หมุนซ้าย: ซ้ายถอย ขวาเดินหน้า
  else      { setMotorL(1);  setMotorR(-1); } // หมุนขวา: ซ้ายเดินหน้า ขวาถอย

  int mode = left ? MODE_SPIN_LEFT : MODE_SPIN_RIGHT;
  unsigned long target_pulses = degToPulses(target_deg);
  unsigned long timeout_ms = computeTimeoutMs(target_pulses);
  unsigned long start_time = millis();

  unsigned long pl = 0, pr = 0;
  while (((pl + pr) / 2) < target_pulses) {
    if (millis() - start_time > timeout_ms) break; // safety timeout

    readEncoders(&pl, &pr);
    updateOdometry(pl, pr, mode);

    int correction = (int)(((long)pl - (long)pr) * KP_BALANCE);
    int base = rampPower(TURN_SPEED, millis() - start_time, (pl + pr) / 2, target_pulses);
    applyPower(base - correction, base + correction);

    printStatus(pl, pr);
    delay(5);
  }

  lockPosition(LOCK_MS, mode); // หมุนเสร็จ: ล็อกตำแหน่ง แล้วนิ่ง LOCK_MS ก่อนออกตัว
}

void around_left(float target_deg)  { spinInPlace(target_deg, true); }
void around_right(float target_deg) { spinInPlace(target_deg, false); }

// หมุนตามเครื่องหมาย: บวก = ซ้าย, ลบ = ขวา, 0 = ไม่หมุน
void turnSigned(float deg) {
  if (deg > 0) around_left(deg);
  else if (deg < 0) around_right(-deg);
}

// ไปยังพิกัด (target_x, target_y) หน่วยเซนติเมตร เทียบจากจุดเริ่มต้น
void goToPoint(float target_x, float target_y) {
  float dx = target_x - pos_x;
  float dy = target_y - pos_y;
  float distance = sqrt(dx * dx + dy * dy);
  float turn_needed = normalizeAngle(atan2(dy, dx) * 180.0 / PI - heading_deg);

  if (turn_needed > 0.5 || turn_needed < -0.5) turnSigned(turn_needed);
  forward(distance);
}

// ============================================================================
// 8) ROUTE RUN
// ============================================================================

// เรียงจุดแก้เอียงจากระยะน้อยไปมาก กันลำดับที่ใส่มาไม่ตรงกับตำแหน่งจริง
void sortCorrections(float *at_cm, float *deg, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (at_cm[j] > at_cm[j + 1]) {
        Serial.println("[runLeg] WARNING: correction points out of order -> swapped");
        float tc = at_cm[j]; at_cm[j] = at_cm[j + 1]; at_cm[j + 1] = tc;
        float td = deg[j];   deg[j]   = deg[j + 1];   deg[j + 1]   = td;
      }
    }
  }
}

// วิ่ง leg เดียว: เดินหน้าเป็นช่วง ๆ หยุดหักแก้เอียงตามจุดที่ตั้งไว้ (สูงสุด 3 จุด, at_cm = 0 คือข้าม)
// แล้วปิดท้ายด้วยสปิน + หมุนปรับทิศ
// (เบรกสั้น ๆ ก่อนหมุน และล็อกตำแหน่งหลังหมุน อยู่ในฟังก์ชันย่อยแล้ว)
void runLeg(float forward_cm,
            float spin_deg, bool spin_right,
            float turn_deg, bool turn_left,
            float corr1_deg, float corr1_at_cm,
            float corr2_deg, float corr2_at_cm,
            float corr3_deg, float corr3_at_cm) {
  float bp_cm[MAX_CORRECTIONS];
  float bp_deg[MAX_CORRECTIONS];
  int n = 0;

  // คัดเฉพาะจุดที่เปิดใช้ (at_cm > 0) และต้องอยู่ก่อนปลาย leg
  if (corr1_at_cm > 0 && corr1_at_cm < forward_cm) { bp_cm[n] = corr1_at_cm; bp_deg[n] = corr1_deg; n++; }
  else if (corr1_at_cm >= forward_cm) Serial.println("[runLeg] WARNING: corr1 >= forward_cm, skipped");
  if (corr2_at_cm > 0 && corr2_at_cm < forward_cm) { bp_cm[n] = corr2_at_cm; bp_deg[n] = corr2_deg; n++; }
  else if (corr2_at_cm >= forward_cm) Serial.println("[runLeg] WARNING: corr2 >= forward_cm, skipped");
  if (corr3_at_cm > 0 && corr3_at_cm < forward_cm) { bp_cm[n] = corr3_at_cm; bp_deg[n] = corr3_deg; n++; }
  else if (corr3_at_cm >= forward_cm) Serial.println("[runLeg] WARNING: corr3 >= forward_cm, skipped");

  sortCorrections(bp_cm, bp_deg, n);

  float dist_so_far = 0;
  for (int i = 0; i < n; i++) {
    float seg = bp_cm[i] - dist_so_far;
    if (seg <= 0) {
      Serial.println("[runLeg] WARNING: correction point <= previous point, skipped");
      continue;
    }
    forward(seg);           // ครบระยะแล้วเบรกนิ่งสั้น ๆ ในตัว
    dist_so_far = bp_cm[i];
    turnSigned(bp_deg[i]);  // เบรกก่อนหมุน + ล็อกตำแหน่งหลังหมุน ในตัว
  }

  float remaining_cm = forward_cm - dist_so_far;
  if (remaining_cm < 0) {
    Serial.println("[runLeg] WARNING: remaining distance negative, clamped to 0");
    remaining_cm = 0;
  }
  forward(remaining_cm);

  if (spin_deg != 0) {
    if (spin_right) around_right(spin_deg); else around_left(spin_deg);
  }
  if (turn_deg != 0) {
    if (turn_left) around_left(turn_deg); else around_right(turn_deg);
  }
}

// LEG 0: หมุนอยู่กับที่ ณ จุดเริ่มต้น ตามค่า LEG0_* ที่ตั้งไว้ แล้วค่อยไปต่อ LEG 1
void runLeg0() {
  if (LEG0_SPIN_DEG == 0 && LEG0_TURN_DEG == 0) return;

  Serial.println("=== LEG 0 (start heading) ===");
  if (LEG0_SPIN_DEG != 0) {
    if (LEG0_SPIN_RIGHT) around_right(LEG0_SPIN_DEG); else around_left(LEG0_SPIN_DEG);
  }
  if (LEG0_TURN_DEG != 0) {
    if (LEG0_TURN_LEFT) around_left(LEG0_TURN_DEG); else around_right(LEG0_TURN_DEG);
  }
}

void setup() {
  Serial.begin(9600);
  motorSetup();
  encoderSetup();

  // ==========================================
  // เส้นทาง 3 leg ปรับค่าที่ค่าคงที่ LEGx_* ด้านบนที่เดียว ไม่ต้องแก้ตรงนี้
  // ความเร็วปรับที่ SPEED / TURN_SPEED ด้านบน
  // ==========================================

  // LEG 0: หมุนตั้งทิศที่จุดเริ่มต้น ก่อนเข้า LEG 1 (เบรกสั้น ๆ ก่อนหมุน + ล็อกตำแหน่งหลังหมุน อยู่ในตัว)
  runLeg0();

  runLeg(LEG1_FORWARD_CM, LEG1_SPIN_DEG, LEG1_SPIN_RIGHT, LEG1_TURN_DEG, LEG1_TURN_LEFT,
         LEG1_CORRECTION_DEG, LEG1_CORRECTION_AT_CM, LEG1_CORRECTION2_DEG, LEG1_CORRECTION2_AT_CM,
         LEG1_CORRECTION3_DEG, LEG1_CORRECTION3_AT_CM);

  runLeg(LEG2_FORWARD_CM, LEG2_SPIN_DEG, LEG2_SPIN_RIGHT, LEG2_TURN_DEG, LEG2_TURN_LEFT,
         LEG2_CORRECTION_DEG, LEG2_CORRECTION_AT_CM, LEG2_CORRECTION2_DEG, LEG2_CORRECTION2_AT_CM,
         LEG2_CORRECTION3_DEG, LEG2_CORRECTION3_AT_CM);

  runLeg(LEG3_FORWARD_CM, LEG3_SPIN_DEG, LEG3_SPIN_RIGHT, LEG3_TURN_DEG, LEG3_TURN_LEFT,
         LEG3_CORRECTION_DEG, LEG3_CORRECTION_AT_CM, LEG3_CORRECTION2_DEG, LEG3_CORRECTION2_AT_CM,
         LEG3_CORRECTION3_DEG, LEG3_CORRECTION3_AT_CM);

  brakeHold(500);
  motorsOff();
  Serial.println("=== ROUTE DONE ===");
}

void loop() {
  // ปล่อยว่างไว้ เพื่อไม่ให้หุ่นยนต์ทำงานวนซ้ำ
}
