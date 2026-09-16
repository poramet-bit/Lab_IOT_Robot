// =====================================================================
// Motor Test - เทสมอเตอร์ซ้าย/ขวาแยกทีละตัว (L298N + LM393 Encoder)
// ใช้ยืนยัน: ขาต่อถูกฝั่ง (ซ้าย/ขวา) + ทิศหมุนถูกทาง (forward/backward)
// หมุนตามจำนวนรอบ (encoder) ไม่ใช่จับเวลา
// ปรับค่าได้ในบล็อก CONFIG ด้านล่าง
// =====================================================================

// ---------------- CONFIG: ขา L298N (ตามของจริงใน robotcurclerun.ino) ----------------
const int ENA = 5, IN1 = 7, IN2 = 6;   // มอเตอร์ซ้าย
const int ENB = 10, IN3 = 8, IN4 = 9;  // มอเตอร์ขวา

// ---------------- CONFIG: ขา Encoder (ตามของจริงใน robotcurclerun.ino) ----------------
const int ENCODER_L = 3;
const int ENCODER_R = 11;
const int DISK_SLOTS = 20; // pulse ต่อ 1 รอบล้อ

// ---------------- CONFIG: ความละเอียด PWM ----------------
const int PWM_RESOLUTION_BITS = 16;          // ตรงกับ robotcurclerun.ino (0-65535)
const int TEST_SPEED = 65000;                // ความเร็วทดสอบ (ปรับตาม PWM_RESOLUTION_BITS)

// ---------------- CONFIG: จำนวนรอบ ----------------
// หมุนรวบเดียวยาวทีเดียวจนครบจำนวนนี้ (ไม่ตัดเป็นหลายรอบสั้นๆ) — แม่นยำกว่า
// เพราะไม่มี error จากการเบรก/สตาร์ทใหม่สะสมทุกรอบ
const float TOTAL_ROUNDS = 10.0;    // จำนวนรอบล้อทั้งหมดที่จะหมุนต่อมอเตอร์ 1 ครั้ง
const unsigned long PAUSE_MS = 1000; // พักระหว่างขั้นตอน (ms)

// Timeout กันค้าง เผื่อ encoder ไม่ขยับ/ล้อติด (ms ต่อ 1 รอบล้อ, คูณ TOTAL_ROUNDS เอง)
const unsigned long TIMEOUT_MS_PER_ROUND = 2000;

volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }

void setup() {
    Serial.begin(9600);
    delay(1000);

    analogWriteResolution(PWM_RESOLUTION_BITS);
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    stopAll();

    pinMode(ENCODER_L, INPUT_PULLUP);
    pinMode(ENCODER_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);

    Serial.println("=== Motor Test Ready ===");
}

bool test_finished = false;

void loop() {
    if (test_finished) return; // ทดสอบครบแล้ว หยุดนิ่ง

    testMotor("LEFT", ENA, IN1, IN2, &pulse_count_L, true);   // forward
    pause();

    testMotor("RIGHT", ENB, IN3, IN4, &pulse_count_R, true);  // forward
    pause();

    test_finished = true;
    Serial.println("=== Test Done ===");
}

void testMotor(const char* label, int enPin, int in1Pin, int in2Pin, volatile unsigned long* pulseCounter, bool forward) {
    Serial.print(label);
    Serial.println(forward ? " FORWARD" : " BACKWARD");

    unsigned long target_pulses = (unsigned long)(TOTAL_ROUNDS * DISK_SLOTS);
    *pulseCounter = 0;

    digitalWrite(in1Pin, forward ? HIGH : LOW);
    digitalWrite(in2Pin, forward ? LOW : HIGH);
    analogWrite(enPin, TEST_SPEED);

    unsigned long timeout_ms = (unsigned long)(TIMEOUT_MS_PER_ROUND * TOTAL_ROUNDS);
    unsigned long start = millis();
    while (*pulseCounter < target_pulses) {
        if (millis() - start >= timeout_ms) {
            Serial.print(label);
            Serial.print(" WARNING: encoder timeout, pulses=");
            Serial.print(*pulseCounter);
            Serial.print("/");
            Serial.println(target_pulses);
            break;
        }
    }

    stopAll();
    Serial.print(label);
    Serial.print(" done, pulses=");
    Serial.println(*pulseCounter);
}

void pause() {
    Serial.println("-- pause --");
    delay(PAUSE_MS);
}

void stopAll() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
