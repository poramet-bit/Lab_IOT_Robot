// ============================================================================
// OPY ROUTE RUNNER — LM393 closed-loop odometry
// - ความเร็วระดับกลางค่อนไปทางสูง (ปรับที่ SPEED / TURN_SPEED)
// - LM393 encoder: นับพัลส์ -> รอบล้อ -> ระยะทาง -> มุมองศาของตัวรถ (heading)
// - ก่อนหมุน และ เมื่อวิ่งครบระยะที่กำหนด: เบรกแล้วนิ่ง 1 วินาที
// - หมุนเสร็จ: ล็อกตำแหน่ง (แก้การไถลหลังหยุด) แล้วนิ่ง 1 วินาทีก่อนออกตัว
// - วิ่งตรง: PI balance ซ้าย-ขวา + แก้มุม heading + ramp ออกตัว/ชะลอก่อนถึงเป้า
// - เส้นทาง 3 leg (LEG1-LEG3)
// - แก้ทิศมอเตอร์ที่ต่อกลับขั้วด้วยธง INVERT_L / INVERT_R
// ============================================================================

// --- กำหนดขามอเตอร์ ---
int ENA = 5;
int IN1 = 6;
int IN2 = 7;

int ENB = 10;
int IN3 = 8;
int IN4 = 9;

// --- ทิศการหมุนของมอเตอร์ ---
// รถวิ่งถอยหลังทั้งที่สั่งเดินหน้า = สายมอเตอร์ต่อกลับขั้ว แก้ที่นี่ที่เดียว ไม่ต้องสลับสายจริง
// true = กลับทิศล้อนั้น. ถ้ากลับทั้งสองล้อ ทิศหมุนซ้าย/ขวาจะถูกแก้ให้ตรงตามจริงไปด้วย
// ถ้าหลังแก้แล้วรถเดินหน้าถูกแต่หมุนผิดข้าง ให้กลับค่าเพียงล้อเดียว
const bool INVERT_L = true;
const bool INVERT_R = true;

// --- ความเร็วรถ เท่ากันทุกจุด ปรับที่นี่ที่เดียว (สเกล PWM 12-bit: 0-4095) ---
// ระดับกลางค่อนไปทางสูง ~70% ของกำลังสูงสุด: แรงพอไม่ตื้อ แต่ไม่พุ่งจนล้อลื่นแล้วเบี้ยว
int SPEED = 2900;      // ความเร็วเดินหน้า/ถอยหลัง
int TURN_SPEED = 2500; // ความเร็วตอนหมุน (ต่ำกว่าเดินหน้าเล็กน้อย เพื่อให้หยุดมุมได้แม่น)

// --- ramp ออกตัว / ชะลอก่อนถึงเป้า (ช่วยให้วิ่งตรงและหยุดตรงจุด) ---
const int START_PWM = 1800;        // PWM ตอนเริ่มออกตัว แล้วไต่ขึ้นหา SPEED
const int FINISH_PWM = 1700;       // PWM ช่วงท้ายก่อนถึงเป้า (ชะลอกันเลยเป้า)
const unsigned long RAMP_UP_MS = 350;   // เวลาที่ใช้ไต่จาก START_PWM ถึง SPEED
const unsigned long DECEL_PULSES = 6;   // เหลือกี่พัลส์ก่อนถึงเป้าจึงเริ่มชะลอ

// --- ตัวแปรปรับแก้ค่าความเร็ว (กรณีล้อวิ่งไม่เท่ากัน) ---
// หมายเหตุ: ค่า trim เดิม 250 ตั้งไว้ที่ SPEED=4000 พอลดความเร็วลงต้องลดตามสัดส่วน
int B_L = 180;
int B_R = 0;

// --- ตัวหักเลี้ยวแก้เอียงระหว่างวิ่งตรง ---
float Kp_balance = 5.0;   // แก้ตามผลต่างพัลส์สด ซ้าย-ขวา (ยิ่งสูงยิ่งแก้ไว แต่ส่ายง่าย)
float Ki_balance = 0.15;  // สะสมผลต่างพัลส์ กันเอียงค้างทางเดียวตลอดเส้น
float Kp_heading = 12.0;  // ดึงกลับหา heading ที่ตั้งไว้ตอนเริ่มวิ่ง (PWM ต่อ 1 องศาเบี้ยว)
const float BALANCE_I_LIMIT = 400.0; // เพดาน integral กัน windup

// --- เวลาเบรก/ล็อกตำแหน่ง ---
const int BRAKE_MS = 1000;     // เบรกนิ่ง 1 วิ ก่อนหมุน และเมื่อวิ่งครบระยะ
const int LOCK_MS = 1000;      // หมุนเสร็จ ล็อกตำแหน่งแล้วนิ่ง 1 วิ ก่อนออกตัว
const unsigned long LOCK_TOL_PULSES = 1;  // ไถลเกินกี่พัลส์จึงดันกลับ
const int LOCK_NUDGE_PWM = 2200;          // แรงดันกลับตอนล็อกตำแหน่ง
const int LOCK_NUDGE_MS = 40;             // เวลาดันกลับต่อครั้ง

// --- เรขาคณิตรถ สำหรับแปลง "องศาที่อยากหมุน" เป็น "จำนวนพัลส์" (ต้องวัดจากรถจริง) ---
const int DISK_SLOTS = 20;              // จำนวนรูบนแผ่น encoder (1 รอบล้อ)
const float WHEEL_DIAMETER_CM = 6.5;    // เส้นผ่านศูนย์กลางล้อ (ซม.) - แก้ตามล้อจริง
const float TRACK_WIDTH_CM = 14.5;      // ระยะห่างล้อซ้าย-ขวา (ซม.) - แก้ตามตัวถังจริง
const float WHEEL_CIRCUMFERENCE_CM = 3.14159265 * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE_THEORY = WHEEL_CIRCUMFERENCE_CM / DISK_SLOTS;
float CM_PER_PULSE_CAL = 1.0; // ตัวชดเชยจากการวัดจริง (ล้อลื่นไถล/เฟืองสูญเสีย)
                               // คาลิเบรต: สั่งวิ่ง 100cm วัดระยะจริง -> เกิน = เพิ่มค่านี้, ขาด = ลดค่านี้
float CM_PER_PULSE = CM_PER_PULSE_THEORY * CM_PER_PULSE_CAL;
float TURN_DEG_SCALE = 1.0; // ตัวชดเชยมุมหมุนจริง - เริ่ม 1.0 แล้วทดสอบจริงค่อยปรับ

// แปลงองศาที่อยากหมุน เป็นจำนวนพัลส์เป้าหมาย (arc length ของล้อที่หมุนรอบจุดกึ่งกลางตัวถัง)
unsigned long degToPulses(float target_deg) {
  return (unsigned long)(((target_deg * TURN_DEG_SCALE) / 360.0) * (3.14159265 * TRACK_WIDTH_CM / CM_PER_PULSE));
}

// --- ตำแหน่ง/ทิศทางรถ (dead-reckoning จากพัลส์ LM393) ---
// origin (0,0) = จุดเริ่มต้นตอนเปิดเครื่อง, heading 0 = ทิศที่รถหันตอนเริ่ม, มุมเพิ่ม = เลี้ยวซ้าย (ทวนเข็ม)
float pos_x = 0.0;
float pos_y = 0.0;
float heading_deg = 0.0;     // มุมองศาของตัวรถ คำนวณสดจาก encoder ทั้งตอนวิ่งตรงและตอนหมุน

float normalizeAngle(float a) {
  while (a > 180.0) a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

// --- ค่าคงที่แต่ละ leg ของเส้นทาง: ระยะวิ่งรวม + จุดหักแก้เอียงกลางทาง (สูงสุด 3 จุด) + หมุนปิดท้าย ---
// CORRECTION_AT_CM = 0 หมายถึง "ไม่ใช้จุดนี้" (ข้าม)
const float LEG1_FORWARD_CM = 320;
const float LEG1_SPIN_DEG = 89.5; const bool LEG1_SPIN_RIGHT = true;  // spin right
const float LEG1_TURN_DEG = 0.0;  const bool LEG1_TURN_LEFT = true;  // turn left
const float LEG1_CORRECTION_DEG = 0;
const float LEG1_CORRECTION_AT_CM = 0.0;
const float LEG1_CORRECTION3_DEG = 0.0;   // จุดเลี้ยวที่ 3 (ยังไม่ใช้)
const float LEG1_CORRECTION3_AT_CM = 0.0; // 0 = ปิดใช้งานจุดนี้

const float LEG2_FORWARD_CM = 400.0;
const float LEG2_SPIN_DEG = 0; const bool LEG2_SPIN_RIGHT = false; // spin left
const float LEG2_TURN_DEG = 0;    const bool LEG2_TURN_LEFT = false; // turn right
const float LEG2_CORRECTION_DEG = 0.0;
const float LEG2_CORRECTION_AT_CM = 0.0;
const float LEG2_CORRECTION2_DEG = 0.0;
const float LEG2_CORRECTION2_AT_CM = 0.0;
const float LEG2_CORRECTION3_DEG = 0.0;
const float LEG2_CORRECTION3_AT_CM = 0.0;

const float LEG3_FORWARD_CM = 860.0;
const float LEG3_SPIN_DEG = 565; const bool LEG3_SPIN_RIGHT = true;  // spin right
const float LEG3_TURN_DEG = 0.0;  const bool LEG3_TURN_LEFT = true;  // turn left
const float LEG3_CORRECTION_DEG = 35;
const float LEG3_CORRECTION_AT_CM = 50.0;
const float LEG3_CORRECTION2_DEG = -10.0;
const float LEG3_CORRECTION2_AT_CM = 400.0;
const float LEG3_CORRECTION3_DEG = 0.0;
const float LEG3_CORRECTION3_AT_CM = 0.0;

// --- LM393 Speed Encoders (นับพัลส์แทนการจับเวลา) ---
const int ENCODER_L = 3;  // ล้อซ้าย (code L)
const int ENCODER_R = 11; // ล้อขวา (code R)
// UNO R4 WiFi ใช้ attachInterrupt ได้หลายขา ไม่จำกัดแค่ 2/3 แต่ถ้าพัลส์ขวาไม่ขึ้น
// ให้ย้าย ENCODER_R ไปขา 2 แล้วเช็ก Serial ว่าค่า R เดินตาม L

volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }

// ตัวนับสะสมตลอดเส้นทาง ใช้รายงาน "รอบล้อ" รวม
unsigned long total_pulses_L = 0;
unsigned long total_pulses_R = 0;

// ค่าพัลส์ครั้งก่อน ใช้คำนวณ odometry แบบเพิ่มทีละก้าว
unsigned long odo_prev_L = 0;
unsigned long odo_prev_R = 0;

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

void readEncoders(unsigned long *l, unsigned long *r) {
  noInterrupts();
  *l = pulse_count_L;
  *r = pulse_count_R;
  interrupts();
}

// รอบล้อสะสม (1 รอบ = DISK_SLOTS พัลส์)
float wheelRevsL() { return (total_pulses_L + pulse_count_L) / (float)DISK_SLOTS; }
float wheelRevsR() { return (total_pulses_R + pulse_count_R) / (float)DISK_SLOTS; }

// --- odometry: อัปเดต heading/x/y จากพัลส์ที่เพิ่มขึ้น ---
// mode 0 = เดินหน้า, 1 = ถอยหลัง, 2 = หมุนซ้ายอยู่กับที่, 3 = หมุนขวาอยู่กับที่
void updateOdometry(unsigned long pl, unsigned long pr, int mode) {
  long dl = (long)pl - (long)odo_prev_L;
  long dr = (long)pr - (long)odo_prev_R;
  odo_prev_L = pl;
  odo_prev_R = pr;
  if (dl == 0 && dr == 0) return;

  float sl = dl * CM_PER_PULSE;
  float sr = dr * CM_PER_PULSE;

  if (mode == 2) { sl = -sl; }        // หมุนซ้าย: ล้อซ้ายถอย ล้อขวาเดินหน้า
  else if (mode == 3) { sr = -sr; }   // หมุนขวา: ล้อซ้ายเดินหน้า ล้อขวาถอย
  else if (mode == 1) { sl = -sl; sr = -sr; } // ถอยหลัง

  float ds = (sl + sr) / 2.0;
  float dth_deg = ((sr - sl) / TRACK_WIDTH_CM) * (180.0 / PI) * TURN_DEG_SCALE;

  float mid_rad = (heading_deg + dth_deg / 2.0) * PI / 180.0;
  pos_x += ds * cos(mid_rad);
  pos_y += ds * sin(mid_rad);
  heading_deg = normalizeAngle(heading_deg + dth_deg);
}

// พิมพ์สถานะออก Serial ทุก 200ms: พัลส์ / รอบล้อ / มุมองศารถ / ตำแหน่ง
void printStatus(unsigned long pl, unsigned long pr) {
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time >= 200) {
    Serial.print("L:");
    Serial.print(pl);
    Serial.print(" R:");
    Serial.print(pr);
    Serial.print(" | rev L:");
    Serial.print(wheelRevsL(), 2);
    Serial.print(" R:");
    Serial.print(wheelRevsR(), 2);
    Serial.print(" | deg:");
    Serial.print(heading_deg, 1);
    Serial.print(" | x:");
    Serial.print(pos_x, 1);
    Serial.print(" y:");
    Serial.println(pos_y, 1);
    last_print_time = millis();
  }
}

// เวลาสูงสุดต่อ 1 พัลส์ ที่ยอมให้รอ (กันหุ่นวิ่งค้าง/ล้อไม่หมุน) - ต้องมากกว่าเวลาจริงต่อพัลส์
// ที่ความเร็วช้าสุดที่เป็นไปได้ ไม่งั้น timeout จะตัดตอนก่อนถึงเป้าในระยะ/มุมที่ยาว
float TIMEOUT_MS_PER_PULSE = 14.0; // ความเร็วลดลงจากเดิม เวลาต่อพัลส์จึงยาวขึ้น
unsigned long MIN_TIMEOUT_MS = 3000;

unsigned long computeTimeoutMs(unsigned long target_pulses) {
  unsigned long t = (unsigned long)(target_pulses * TIMEOUT_MS_PER_PULSE);
  return t < MIN_TIMEOUT_MS ? MIN_TIMEOUT_MS : t;
}

// กำลังขับ ณ ขณะนั้น: ไต่ขึ้นตอนออกตัว และชะลอลงก่อนถึงเป้า
int rampPower(int cruise, unsigned long elapsed_ms, unsigned long done_pulses, unsigned long target_pulses) {
  int p = cruise;
  if (elapsed_ms < RAMP_UP_MS) {
    p = START_PWM + (int)((long)(cruise - START_PWM) * (long)elapsed_ms / (long)RAMP_UP_MS);
  }
  unsigned long remaining = (target_pulses > done_pulses) ? (target_pulses - done_pulses) : 0;
  if (remaining <= DECEL_PULSES) {
    int q = FINISH_PWM + (int)((long)(cruise - FINISH_PWM) * (long)remaining / (long)DECEL_PULSES);
    if (q < p) p = q;
  }
  return p;
}

void setup() {
  Serial.begin(9600);
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

  analogWriteResolution(12); // กำหนดค่า PWM 0-4095 (สำหรับบอร์ด Arduino UNO R4 / ESP32)

  // ==========================================
  // เส้นทางแบ่งเป็น 3 leg ปรับค่าได้ที่ค่าคงที่ LEGx_* ด้านบนที่เดียว
  // ความเร็วปรับที่ SPEED/TURN_SPEED ด้านบน เท่ากันทุกจุดทั้งเส้นทาง
  // ==========================================
  brakeHold(2000);

  runLeg(LEG1_FORWARD_CM, LEG1_SPIN_DEG, LEG1_SPIN_RIGHT, LEG1_TURN_DEG, LEG1_TURN_LEFT,
         LEG1_CORRECTION_DEG, LEG1_CORRECTION_AT_CM, 0, 0,
         LEG1_CORRECTION3_DEG, LEG1_CORRECTION3_AT_CM);

  runLeg(LEG2_FORWARD_CM, LEG2_SPIN_DEG, LEG2_SPIN_RIGHT, LEG2_TURN_DEG, LEG2_TURN_LEFT,
         LEG2_CORRECTION_DEG, LEG2_CORRECTION_AT_CM, LEG2_CORRECTION2_DEG, LEG2_CORRECTION2_AT_CM,
         LEG2_CORRECTION3_DEG, LEG2_CORRECTION3_AT_CM);

  runLeg(LEG3_FORWARD_CM, LEG3_SPIN_DEG, LEG3_SPIN_RIGHT, LEG3_TURN_DEG, LEG3_TURN_LEFT,
         LEG3_CORRECTION_DEG, LEG3_CORRECTION_AT_CM, LEG3_CORRECTION2_DEG, LEG3_CORRECTION2_AT_CM,
         LEG3_CORRECTION3_DEG, LEG3_CORRECTION3_AT_CM);

  brakeHold(500);
  motorsOff();
  // จบการทำงาน
}

void loop() {
  // ปล่อยว่างไว้ เพื่อไม่ให้หุ่นยนต์ทำงานวนซ้ำ
}

// -------------------------------------------------------------
// เบรก / หยุด / ล็อกตำแหน่ง
// -------------------------------------------------------------

// สั่งทิศล้อซ้าย: dir 1 = เดินหน้า, -1 = ถอยหลัง, 0 = เบรก (ผ่านธง INVERT_L)
void setMotorL(int dir) {
  int d = INVERT_L ? -dir : dir;
  if (d > 0)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (d < 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else            { digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); }
}

// สั่งทิศล้อขวา: dir 1 = เดินหน้า, -1 = ถอยหลัง, 0 = เบรก (ผ่านธง INVERT_R)
void setMotorR(int dir) {
  int d = INVERT_R ? -dir : dir;
  if (d > 0)      { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else if (d < 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else            { digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); }
}

// เบรกจริง (short brake: ขา IN ทั้งคู่เท่ากัน + EN เต็ม) หยุดเฉียบกว่าปล่อยไหล
void brakeMotors() {
  setMotorL(0);
  setMotorR(0);
  analogWrite(ENA, 4095);
  analogWrite(ENB, 4095);
}

// ตัดไฟมอเตอร์ (ปล่อยอิสระ) ใช้ตอนจบงานเท่านั้น
void motorsOff() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

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

// ล็อกตำแหน่งหลังหมุนเสร็จ: เบรกค้าง คอยจับพัลส์ที่ไถลต่อ แล้วดันกลับให้มุมคงเดิม
// mode 2 = เพิ่งหมุนซ้าย, mode 3 = เพิ่งหมุนขวา (ใช้เลือกทิศดันกลับ)
void lockPosition(int time_ms, int mode) {
  brakeMotors();
  unsigned long pl, pr;
  readEncoders(&pl, &pr);
  updateOdometry(pl, pr, mode);
  unsigned long base_l = pl;
  unsigned long base_r = pr;

  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)time_ms) {
    readEncoders(&pl, &pr);
    updateOdometry(pl, pr, mode);
    unsigned long drift = ((pl - base_l) + (pr - base_r)) / 2;

    if (drift > LOCK_TOL_PULSES) {
      // ไถลเลยไปในทิศที่เพิ่งหมุน -> ขับสวนสั้น ๆ ดึงกลับ แล้วเบรกค้างต่อ
      if (mode == 2) {                    // เพิ่งหมุนซ้าย -> ดันกลับเป็นหมุนขวา
        setMotorL(1);  setMotorR(-1);
      } else {                            // เพิ่งหมุนขวา -> ดันกลับเป็นหมุนซ้าย
        setMotorL(-1); setMotorR(1);
      }
      analogWrite(ENA, LOCK_NUDGE_PWM);
      analogWrite(ENB, LOCK_NUDGE_PWM);
      delay(LOCK_NUDGE_MS);
      brakeMotors();
      readEncoders(&pl, &pr);
      updateOdometry(pl, pr, (mode == 2) ? 3 : 2); // พัลส์ช่วงดันกลับนับเป็นทิศตรงข้าม
      base_l = pl;
      base_r = pr;
    }

    printStatus(pl, pr);
    delay(5);
  }
}

// -------------------------------------------------------------
// ฟังก์ชันควบคุมการเคลื่อนที่แบบระบุระยะทาง (distance_cm) แทนเวลา
// -------------------------------------------------------------

// วิ่งตรงด้วย closed-loop: บาลานซ์พัลส์ซ้าย-ขวา (PI) + ดึงกลับหา heading เดิม + ramp/ชะลอ
// reverse = true คือถอยหลัง
void driveStraight(float distance_cm, bool reverse) {
  if (distance_cm <= 0) return;
  resetEncoders();

  int dir = reverse ? -1 : 1;
  setMotorL(dir);
  setMotorR(dir);

  unsigned long target_pulses = (unsigned long)(distance_cm / CM_PER_PULSE);
  unsigned long timeout_ms = computeTimeoutMs(target_pulses);
  float heading_ref = heading_deg;   // มุมที่ต้องรักษาไว้ตลอดช่วงวิ่งตรง
  float i_term = 0.0;
  unsigned long start_time = millis();

  unsigned long pl = 0, pr = 0;
  while (((pl + pr) / 2) < target_pulses) {
    if (millis() - start_time > timeout_ms) break; // safety timeout กันค้างถ้านับพัลส์พลาด

    readEncoders(&pl, &pr);
    updateOdometry(pl, pr, reverse ? 1 : 0);

    long err = (long)pl - (long)pr;                       // ซ้ายเกิน = error บวก
    i_term += err * 0.01;
    i_term = constrain(i_term, -BALANCE_I_LIMIT, BALANCE_I_LIMIT);
    float head_err = normalizeAngle(heading_deg - heading_ref); // เบี้ยวซ้าย = บวก
    if (reverse) head_err = -head_err;

    int correction = (int)(err * Kp_balance + i_term * Ki_balance + head_err * Kp_heading);
    int base = rampPower(SPEED, millis() - start_time, (pl + pr) / 2, target_pulses);

    analogWrite(ENA, constrain(base - B_L - correction, 0, 4095));
    analogWrite(ENB, constrain(base - B_R + correction, 0, 4095));

    printStatus(pl, pr);
    delay(5);
  }

  readEncoders(&pl, &pr);
  updateOdometry(pl, pr, reverse ? 1 : 0);
  brakeHold(BRAKE_MS); // ถึงระยะที่กำหนด -> เบรกแล้วนิ่ง 1 วินาที
}

void forward(float distance_cm) {
  driveStraight(distance_cm, false);
}

void backward(float distance_cm) {
  driveStraight(distance_cm, true);
}

// หมุนอยู่กับที่ด้วย closed-loop: บังคับให้สองล้อหมุนเท่ากัน (ไม่ใส่ trim B_L/B_R)
// left = true หมุนซ้าย (ทวนเข็ม), false หมุนขวา
void spinInPlace(float target_deg, bool left) {
  if (target_deg <= 0) return;
  brakeHold(BRAKE_MS); // ก่อนหมุน: เบรกแล้วนิ่ง 1 วินาที
  resetEncoders();

  if (left) {
    setMotorL(-1); setMotorR(1);  // หมุนซ้าย: ล้อซ้ายถอยหลัง ล้อขวาเดินหน้า
  } else {
    setMotorL(1);  setMotorR(-1); // หมุนขวา: ล้อซ้ายเดินหน้า ล้อขวาถอยหลัง
  }

  int mode = left ? 2 : 3;
  unsigned long target_pulses = degToPulses(target_deg);
  unsigned long timeout_ms = computeTimeoutMs(target_pulses);
  unsigned long start_time = millis();

  unsigned long pl = 0, pr = 0;
  while (((pl + pr) / 2) < target_pulses) {
    if (millis() - start_time > timeout_ms) break;

    readEncoders(&pl, &pr);
    updateOdometry(pl, pr, mode);

    long err = (long)pl - (long)pr;
    int correction = (int)(err * Kp_balance);
    int base = rampPower(TURN_SPEED, millis() - start_time, (pl + pr) / 2, target_pulses);

    analogWrite(ENA, constrain(base - correction, 0, 4095));
    analogWrite(ENB, constrain(base + correction, 0, 4095));

    printStatus(pl, pr);
    delay(5);
  }

  lockPosition(LOCK_MS, mode); // หมุนเสร็จ: ล็อกตำแหน่ง แล้วนิ่ง 1 วินาทีก่อนออกตัว
}

void around_left(float target_deg) {
  spinInPlace(target_deg, true);
}

void around_right(float target_deg) {
  spinInPlace(target_deg, false);
}

// ไปยังจุดหมาย (target_x, target_y) หน่วยเซนติเมตร เทียบจากจุดเริ่มต้น
void goToPoint(float target_x, float target_y) {
  float dx = target_x - pos_x;
  float dy = target_y - pos_y;
  float distance = sqrt(dx * dx + dy * dy);
  float target_heading = atan2(dy, dx) * 180.0 / PI;
  float turn_needed = normalizeAngle(target_heading - heading_deg);

  if (turn_needed > 0.5) {
    around_left(turn_needed);
  } else if (turn_needed < -0.5) {
    around_right(-turn_needed);
  }

  forward(distance);
}

// วิ่ง leg เดียว: เดินหน้ารวม forward_cm, หักแก้เอียงกลางทางได้สูงสุด 3 จุด (at_cm=0 = ข้าม),
// จบด้วยหมุนสปิน + หมุนปรับทิศ (ทุกครั้งที่หมุน: เบรก 1 วิ ก่อน / ล็อกตำแหน่ง + นิ่ง 1 วิ หลัง)
void runLeg(float forward_cm,
            float spin_deg, bool spin_right,
            float turn_deg, bool turn_left,
            float corr1_deg, float corr1_at_cm,
            float corr2_deg, float corr2_at_cm,
            float corr3_deg, float corr3_at_cm) {
  float bp_cm[3];
  float bp_deg[3];
  int n = 0;
  if (corr1_at_cm > 0) { bp_cm[n] = corr1_at_cm; bp_deg[n] = corr1_deg; n++; }
  if (corr2_at_cm > 0) { bp_cm[n] = corr2_at_cm; bp_deg[n] = corr2_deg; n++; }
  if (corr3_at_cm > 0) { bp_cm[n] = corr3_at_cm; bp_deg[n] = corr3_deg; n++; }
  // เรียง bp_cm[] จากน้อยไปมาก กันลำดับที่ใส่มาไม่ตรงกับตำแหน่งจริง
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (bp_cm[j] > bp_cm[j + 1]) {
        Serial.println("[runLeg] WARNING: correction points out of order -> swapped to match actual position");
        float tc = bp_cm[j]; bp_cm[j] = bp_cm[j + 1]; bp_cm[j + 1] = tc;
        float td = bp_deg[j]; bp_deg[j] = bp_deg[j + 1]; bp_deg[j + 1] = td;
      }
    }
  }

  float dist_so_far = 0;
  for (int i = 0; i < n; i++) {
    float seg = bp_cm[i] - dist_so_far;
    if (seg <= 0) {
      Serial.println("[runLeg] WARNING: correction point <= previous point, skipped (would be zero/negative distance)");
      continue;
    }
    if (bp_cm[i] >= forward_cm) {
      Serial.println("[runLeg] WARNING: correction point >= forward_cm, skipped");
      continue;
    }
    forward(seg);            // ครบระยะแล้วเบรกนิ่ง 1 วิ ในตัว
    dist_so_far = bp_cm[i];
    if (bp_deg[i] > 0) around_left(bp_deg[i]);       // เบรกก่อนหมุน + ล็อกตำแหน่งหลังหมุน ในตัว
    else if (bp_deg[i] < 0) around_right(-bp_deg[i]);
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
