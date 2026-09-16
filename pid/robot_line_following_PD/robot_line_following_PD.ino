// =====================================================================
// Line Following Robot - PD Controller
// TCRT5000 5CH + L298N + Arduino UNO WiFi R4
// ปรับค่าทั้งหมดได้ในส่วน CONFIG ด้านล่าง
// =====================================================================

// ---------------- CONFIG: TEST MODE ----------------
// 0 = วิ่งจริงด้วย PD
// 1 = ทดสอบเซนเซอร์อย่างเดียว (ไม่หมุนมอเตอร์)  -> แผนข้อ 8 ขั้นที่ 1
// 2 = ทดสอบ Position() อย่างเดียว (ไม่หมุนมอเตอร์) -> แผนข้อ 8 ขั้นที่ 2
// 3 = ทดสอบมอเตอร์ตรง ๆ (หมุนคงที่ ไม่อ่านเซนเซอร์) -> แผนข้อ 8 ขั้นที่ 3
#define TEST_MODE 0

// ---------------- CONFIG: ขา L298N ----------------
// มอเตอร์สลับขากันทางฮาร์ดแวร์ สลับ group พิน+invert คู่กันในโค้ดให้ตรงตามนั้น
const int ENA = 5,  IN1 = 7, IN2 = 6; // มอเตอร์ซ้าย
const int ENB = 10, IN3 = 8, IN4 = 9; // มอเตอร์ขวา

// สลับทิศได้ที่นี่ถ้าต่อสายมอเตอร์กลับด้าน (ไม่ต้องแก้ IN1..IN4)
const bool INVERT_LEFT  = false;

const bool INVERT_RIGHT = true;
// ---------------- CONFIG: ขา TCRT5000 5CH ----------------
// เรียงจากซ้ายไปขวา S1..S5
const int SENSOR_PINS[5] = {A0 ,A1, A2, A3, A4};

// อ่านเซนเซอร์แบบ analog + คาลิเบรต (ย้ายมาจาก robot_line_state_machine)
// FLOOR_RAW = ค่า analogRead ตอนอยู่บนพื้น, BLACK_RAW = ค่าตอนอยู่บนเส้นดำ
// ไม่ต้องสนใจว่าดำแล้วค่าสูงหรือต่ำ สูตร darkness กลับขั้วให้เองจากสองค่านี้ (ไม่ต้องมี BLACK_IS_HIGH แยกอีก)
// ค่าด้านล่างเป็น placeholder เริ่มต้น ไม่ต้องแก้เองแล้ว — คาลิเบรตผ่าน Serial ได้เลย (ดูฟังก์ชัน serviceCalibration)
// พิมพ์ 'f' ตอนรถอยู่บนพื้น (ไม่เจอเส้น) แล้วพิมพ์ 'b' ตอนรถอยู่บนเส้นดำ จะจับค่าจริงต่อช่องให้เอง
// พิมพ์ 'p' เพื่อดูค่าที่คาลิเบรตไว้ตอนนี้
int FLOOR_RAW[5] = {3000, 3000, 3000, 3000, 3000};
int BLACK_RAW[5] = {900, 900, 900, 900, 900};
const int LINE_THRESHOLD = 450; // normalized darkness 0..1000, floor=0 black=1000

const int WEIGHTS[5] = {1000, 2000, 3000, 4000, 5000};
const int SETPOINT   = 3000; // ค่ากลาง (ควรอยู่กึ่งกลาง WEIGHTS)

// ---------------- CONFIG: ความเร็ว/PWM ----------------
int   BaseSpeed = 230;   // ความเร็วพื้นฐาน
float Kp        = 0.15; // ค่า P (ลองขึ้นจาก 0.15 — ยังไม่ได้ทดสอบจริง ต้องวิ่งบนสนามจริงแล้วปรับต่อ)
float Kd        = 0.25; // ค่า D (ขึ้นตาม Kp กันส่าย/overshoot)

const int PWM_MIN = -255;
const int PWM_MAX  = 255;

// ---------------- CONFIG: ลดความเร็วตอนเข้าโค้ง ----------------
// true = error มาก (โค้งแรง) ลด BaseSpeed ลงอัตโนมัติ, ตรงยังวิ่งเร็วเต็ม
const bool CURVE_SLOWDOWN_ENABLE = true;
const int  MIN_CURVE_SPEED   = 220;   // ความเร็วต่ำสุดตอนโค้งแรงสุด
const int  MAX_ERROR_FOR_SCALE = 2000; // |error| ที่ถือว่าโค้งแรงสุด (อิงจากช่วง WEIGHTS)

// ---------------- CONFIG: โค้งหักศอก 90 องศา ----------------
// true = ถ้าเซนเซอร์นอกสุด 2 ตัวฝั่งเดียวติดพร้อมกัน (กลางไม่ติด) ถือว่าเจอโค้ง 90
// สั่ง pivot หมุนแรงทันที แทนคำนวณ error/PD ตามปกติ (ค่า error แบบถ่วงน้ำหนักไม่พอให้เลี้ยวทัน เพราะเส้นหักฉับพลัน ไม่ค่อยๆโค้ง)
const bool CORNER_90_ENABLE   = false; // ปิดไว้ก่อน: เงื่อนไขนี้ดันไปติดตอนวิ่งกลางเส้นปกติด้วย ไม่ใช่แค่โค้ง 90 จริง ต้องหาลายเซ็นเซนเซอร์ที่แม่นกว่านี้จากข้อมูลจริงก่อนเปิดใช้
const int  CORNER_PIVOT_SPEED = 250; // ล้อฝั่งที่ต้องหมุนออก วิ่งเร็วเท่านี้ตอนเจอโค้ง
const int  CORNER_PIVOT_BRAKE = 200; // ล้อฝั่งในโค้ง ถอยเบาๆช่วยหมุนไว (ใส่ 0 ถ้าไม่อยากถอย)

// ---------------- CONFIG: เส้นหลุด ----------------
// true  = ถ้าไม่มีเซนเซอร์ตัวใดเจอเส้น ให้เลี้ยวหาเส้นต่อ (error ค่อยๆโตขึ้นตามเวลาที่หลุด)
// false = ถ้าเส้นหลุด ให้หยุดมอเตอร์ทันที
const bool HOLD_LAST_ERROR_ON_LOST_LINE = false; // false: เส้นหลุดจริงให้หยุดนิ่งแทนถอยหาเส้น (ห้ามแก้กลับเป็น true)

// เส้นหลุดนานเกินนี้ (ms) ให้หยุดมอเตอร์เสมอ กันรถวิ่งวนหลุดสนามไม่รู้จบ
const unsigned long LINE_LOST_STOP_MS = 2000;

// ตอนเส้นหลุด ยิ่งหลุดนาน ยิ่งเลี้ยวแรงขึ้น (หน่วย error ต่อ ms)
const float LOST_LINE_ERROR_GROWTH_PER_MS = 10;

// เพดาน error ตอนเส้นหลุด กันเลี้ยวแรงเกินไป
const int LOST_LINE_ERROR_MAX = 4000;

// เส้นหลุด: ถอยหลังหาเส้นแทนวิ่งหน้าต่อ (เอียงตามทิศที่เห็นเส้นครั้งล่าสุด)
const int SEARCH_REVERSE_SPEED = 2500; // ความเร็วถอยหลังตอนหาเส้น
const int SEARCH_TURN_BIAS     = 70;  // เอียงซ้าย/ขวาขณะถอย กวาดหาเส้นกลับ

// ---------------- CONFIG: Debug ----------------
const bool DEBUG = true;
const unsigned long DEBUG_INTERVAL_MS = 500;

// =====================================================================
// ตัวแปรภายใน
// =====================================================================
int last_error = 0;
bool line_seen_last = true;
unsigned long line_lost_since_ms = 0;
int lost_error_base = 0;
unsigned long last_debug_ms = 0;
int line_lost_streak = 0;
const int LINE_LOST_DEBOUNCE_READS = 4; // อ่านไม่เจอเส้นติดกันกี่ครั้งถึงจะถือว่าหลุดจริง กันเซนเซอร์กระตุกสั่งถอยเอง
float smoothed_position = 3000; // เริ่มที่ SETPOINT
const float POSITION_SMOOTHING = 0.35; // 0..1 ยิ่งน้อยยิ่งกรองแรง/หน่วงมากขึ้น

void setup() {
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    for (int i = 0; i < 5; i++) {
        pinMode(SENSOR_PINS[i], INPUT); // analog ไม่ใช้ pull-up
    }
    analogReadResolution(12);

    Serial.begin(115200);
}

void loop() {
    serviceCalibration(); // พิมพ์ f/b/p ทาง Serial คาลิเบรตเซนเซอร์ได้ตลอด ไม่ต้องสลับโหมด
#if TEST_MODE == 1
    testSensors();
#elif TEST_MODE == 2
    testPosition();
#elif TEST_MODE == 3
    testMotors();
#else
    runLineFollow();
#endif
}

// =====================================================================
// โหมดวิ่งจริง
// =====================================================================
void runLineFollow() {
    PD(BaseSpeed, Kp, Kd);
}

// อ่านเซนเซอร์ 1 ช่องแบบ analog, median ของ 3 ครั้งกันสัญญาณรบกวนจาก L298N
int readSensorRaw(int pin) {
    analogRead(pin); // ทิ้งค่าแรกหลังสลับช่อง ADC
    int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    return a > b ? a : b; // median
}

int lastRaw[5] = {}; // เก็บค่า analogRead ดิบล่าสุดไว้ดูตอนคาลิเบรต (ผ่าน TEST_MODE=1)

// คาลิเบรตเซนเซอร์สดผ่าน Serial ไม่ต้องเดา/แก้ค่าในโค้ดเอง:
// 'f' = วางรถบนพื้น (ไม่เจอเส้น) แล้วพิมพ์ 'f' จับ FLOOR_RAW ทั้ง 5 ช่อง
// 'b' = วางรถให้ทุกเซนเซอร์ทับเส้นดำ แล้วพิมพ์ 'b' จับ BLACK_RAW ทั้ง 5 ช่อง
// 'p' = พิมพ์ค่า FLOOR_RAW/BLACK_RAW ที่คาลิเบรตไว้ตอนนี้
void serviceCalibration() {
    if (!Serial.available()) return;
    int ch = Serial.read();

    if (ch == 'f' || ch == 'F') {
        for (int i = 0; i < 5; i++) FLOOR_RAW[i] = readSensorRaw(SENSOR_PINS[i]);
        Serial.print("FLOOR_RAW ");
        for (int i = 0; i < 5; i++) { Serial.print(FLOOR_RAW[i]); Serial.print(' '); }
        Serial.println();
    } else if (ch == 'b' || ch == 'B') {
        for (int i = 0; i < 5; i++) BLACK_RAW[i] = readSensorRaw(SENSOR_PINS[i]);
        Serial.print("BLACK_RAW ");
        for (int i = 0; i < 5; i++) { Serial.print(BLACK_RAW[i]); Serial.print(' '); }
        Serial.println();
    } else if (ch == 'p' || ch == 'P') {
        Serial.print("FLOOR_RAW "); for (int i = 0; i < 5; i++) { Serial.print(FLOOR_RAW[i]); Serial.print(' '); }
        Serial.print("| BLACK_RAW "); for (int i = 0; i < 5; i++) { Serial.print(BLACK_RAW[i]); Serial.print(' '); }
        Serial.println();
    }
}

// อ่านเซนเซอร์ทั้ง 5 ช่อง คืนค่า true ถ้าเจอเส้นดำ (เทียบ darkness ที่คาลิเบรตแล้วกับ LINE_THRESHOLD)
void readSensors(bool onLine[5]) {
    for (int i = 0; i < 5; i++) {
        int raw = readSensorRaw(SENSOR_PINS[i]);
        lastRaw[i] = raw;
        int span = BLACK_RAW[i] - FLOOR_RAW[i];
        float darkness = span == 0 ? 0 : 1000.0f * (raw - FLOOR_RAW[i]) / span;
        darkness = constrain(darkness, 0, 1000);
        onLine[i] = (darkness >= LINE_THRESHOLD);
    }
}

// เจอโค้ง 90 มั้ย: เซนเซอร์นอกสุด 2 ตัวฝั่งเดียวติด แต่ตัวกลางไม่ติด
// (เส้นตรง/โค้งค่อยเป็นค่อยไป ตัวกลางจะยังติดอยู่ด้วยเสมอ ต่างจากโค้งหักศอก)
// คืนค่า -1 = โค้งซ้าย, 1 = โค้งขวา, 0 = ไม่ใช่โค้ง 90
int cornerDirection(bool onLine[5]) {
    bool leftCorner  = onLine[0] && onLine[1] && !onLine[2];
    bool rightCorner = onLine[4] && onLine[3] && !onLine[2];
    if (leftCorner && !rightCorner) return -1;
    if (rightCorner && !leftCorner) return 1;
    return 0;
}

// คำนวณตำแหน่งเส้นแบบ weighted average
// คืนค่า Position (ตาม WEIGHTS) และปรับ line_seen_last
int Position() {
    bool onLine[5];
    readSensors(onLine);

    float weightedSum = 0;
    int activeCount = 0;

    for (int i = 0; i < 5; i++) {
        if (onLine[i]) {
            weightedSum += WEIGHTS[i];
            activeCount++;
        }
    }

    if (activeCount == 0) {
        line_lost_streak++;
        if (line_lost_streak < LINE_LOST_DEBOUNCE_READS) {
            // ยังไม่ถือว่าหลุดจริง (กันเซนเซอร์กระตุกสั่งถอย) คงตำแหน่งเดิมไว้ก่อน
            return SETPOINT + last_error;
        }

        if (line_seen_last) {
            // เพิ่งหลุดเส้นจริงรอบนี้ จำจุดเริ่มต้นไว้
            line_lost_since_ms = millis();
            lost_error_base = abs(last_error);
        }
        line_seen_last = false;

        int sign = (last_error >= 0) ? 1 : -1;
        unsigned long lostForMs = millis() - line_lost_since_ms;
        int grown = lost_error_base + (int)(LOST_LINE_ERROR_GROWTH_PER_MS * lostForMs);
        grown = min(grown, LOST_LINE_ERROR_MAX);

        return SETPOINT + sign * grown;
    }

    line_lost_streak = 0;
    line_seen_last = true;

    int rawPosition = (int)round(weightedSum / activeCount);
    // กรองสัญญาณ: A0-2 บางทีติดๆดับๆ ทำให้ Position กระโดดจาก ~4000-5000 (เหลือ S4,S5)
    // ไปแถว 1000-3000 ทันทีเป็นเฟรมเดียว แล้ว error เปลี่ยนฮวบฮาบจน Kd/ล้อโดนกดถอย
    // ผสมค่าเก่ากับค่าใหม่แทนใช้ดิบๆ ตัดกระโดดฉับพลันจากอ่านผิดปกติแค่เฟรมเดียว
    smoothed_position = smoothed_position + POSITION_SMOOTHING * (rawPosition - smoothed_position);
    return (int)round(smoothed_position);
}

// คำนวณ PD แล้วสั่งมอเตอร์
void PD(int baseSpeed, float kp, float kd) {
    if (CORNER_90_ENABLE) {
        bool onLine[5];
        readSensors(onLine);
        int corner = cornerDirection(onLine);
        if (corner != 0) {
            int leftMotor  = corner < 0 ? CORNER_PIVOT_BRAKE : CORNER_PIVOT_SPEED;
            int rightMotor = corner < 0 ? CORNER_PIVOT_SPEED : CORNER_PIVOT_BRAKE;
            motors(leftMotor, rightMotor);
            // จำทิศไว้เป็น error สุดขั้ว กัน Kd กระชากตอนกลับเข้า PD ปกติ
            last_error = corner < 0 ? -MAX_ERROR_FOR_SCALE : MAX_ERROR_FOR_SCALE;
            line_seen_last = true;
            if (DEBUG && millis() - last_debug_ms >= DEBUG_INTERVAL_MS) {
                last_debug_ms = millis();
                Serial.print("[CORNER90] dir="); Serial.println(corner);
            }
            return;
        }
    }

    int position = Position();
    int error = position - SETPOINT;

    if (!line_seen_last) {
        unsigned long lostForMs = millis() - line_lost_since_ms;
        bool giveUp = !HOLD_LAST_ERROR_ON_LOST_LINE || (lostForMs >= LINE_LOST_STOP_MS);
        if (giveUp) {
            motors(0, 0);
            return;
        }

        // เส้นหลุด: ถอยหลังหาเส้น เอียงตามทิศ error ล่าสุดที่ยังเห็นเส้นอยู่
        // (ไม่แตะ last_error ที่นี่ — เก็บค่าจริงล่าสุดไว้ ไม่ให้ error สังเคราะห์ตอนหลุด
        // ไปทำให้ Kd กระชากตอนเจอเส้นกลับมา)
        int sign = (last_error >= 0) ? 1 : -1;
        int leftMotor  = -SEARCH_REVERSE_SPEED - sign * SEARCH_TURN_BIAS;
        int rightMotor = -SEARCH_REVERSE_SPEED + sign * SEARCH_TURN_BIAS;
        leftMotor  = constrain(leftMotor, PWM_MIN, PWM_MAX);
        rightMotor = constrain(rightMotor, PWM_MIN, PWM_MAX);
        motors(leftMotor, rightMotor);

        if (DEBUG && millis() - last_debug_ms >= DEBUG_INTERVAL_MS) {
            last_debug_ms = millis();
            Serial.print("[SEARCH] reversing lastErr="); Serial.println(last_error);
        }
        return;
    }

    float output = (error * kp) + ((error - last_error) * kd);
    last_error = error;

    int effectiveBaseSpeed = baseSpeed;
    if (CURVE_SLOWDOWN_ENABLE) {
        int absErr = constrain(abs(error), 0, MAX_ERROR_FOR_SCALE);
        effectiveBaseSpeed = map(absErr, 0, MAX_ERROR_FOR_SCALE, baseSpeed, MIN_CURVE_SPEED);
    }

    // รถเลี้ยวไปทางล้อที่ช้ากว่า: error ติดลบ (เส้น/มุมอยู่ซ้าย) ต้องช้าที่ล้อซ้าย เร็วที่ล้อขวา
    // ของเดิมสลับเครื่องหมาย เจอมุมแรงๆ เลยเร่งล้อฝั่งเดียวกับมุม กลายเป็นหมุนหนีออกจากเส้น
    int leftMotor  = effectiveBaseSpeed + (int)output;
    int rightMotor = effectiveBaseSpeed - (int)output;

    // ตอนวิ่งตามเส้นปกติ ห้ามล้อถอยหลัง (Kp/Kd สูง output พุ่งเกิน effectiveBaseSpeed
    // ได้ง่าย ล้อฝั่งหนึ่งติดลบกลายเป็นถอยหลังทั้งที่เจอเส้นอยู่) กดพื้นล่างไว้ที่ 0
    leftMotor  = constrain(leftMotor, 0, PWM_MAX);
    rightMotor = constrain(rightMotor, 0, PWM_MAX);

    motors(leftMotor, rightMotor);

    if (DEBUG && millis() - last_debug_ms >= DEBUG_INTERVAL_MS) {
        last_debug_ms = millis();
        Serial.print("Pos="); Serial.print(position);
        Serial.print(" Err="); Serial.print(error);
        Serial.print(" Out="); Serial.print(output);
        Serial.print(" Spd="); Serial.print(effectiveBaseSpeed);
        Serial.print(" L="); Serial.print(leftMotor);
        Serial.print(" R="); Serial.println(rightMotor);
    }
}

// ควบคุม L298N ด้วยค่า -255..255 (ลบ = ถอยหลัง)
void motors(int leftMotor, int rightMotor) {
    setMotor(leftMotor,  INVERT_LEFT,  ENA, IN1, IN2);
    setMotor(rightMotor, INVERT_RIGHT, ENB, IN3, IN4);
}

void setMotor(int speed, bool invert, int enPin, int in1Pin, int in2Pin) {
    if (invert) speed = -speed;
    speed = constrain(speed, PWM_MIN, PWM_MAX);

    bool forward = speed >= 0;
    digitalWrite(in1Pin, forward ? HIGH : LOW);
    digitalWrite(in2Pin, forward ? LOW  : HIGH);
    analogWrite(enPin, abs(speed));
}

// =====================================================================
// โหมดทดสอบ (แผนข้อ 8)
// =====================================================================
void testSensors() {
    bool onLine[5];
    readSensors(onLine);
    for (int i = 0; i < 5; i++) {
        Serial.print("S"); Serial.print(i + 1); Serial.print("=");
        Serial.print(onLine[i] ? "1" : "0");
        Serial.print("(raw="); Serial.print(lastRaw[i]); Serial.print(") ");
    }
    Serial.println();
    delay(200);
}

void testPosition() {
    int position = Position();
    Serial.print("Position="); Serial.println(position);
    delay(200);
}

void testMotors() {
    // ทดสอบทีละล้อ แยกดูว่าพิน ENA=ล้อไหนจริง ๆ (สลับพินหรือแค่ทิศกลับขั้ว)
    Serial.println("LEFT (ENA) forward only");
    motors(BaseSpeed, 0);
    delay(500);
    motors(0, 0);
    delay(800);

    Serial.println("RIGHT (ENB) forward only");
    motors(0, BaseSpeed);
    delay(500);
    motors(0, 0);
    delay(2000);
}
