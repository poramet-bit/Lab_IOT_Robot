#pragma once

// แก้ค่าตรงนี้ก่อน — วัดจากเซนเซอร์จริงบนรถคันนี้ (S1..S5 = A0..A4 ซ้าย->ขวา)
// FLOOR_RAW = ค่า analogRead ตอนอยู่บนพื้น (ไม่เจอเส้นดำ)
// BLACK_RAW = ค่า analogRead ตอนอยู่บนเส้นดำ
// ไม่ต้องสนใจว่าดำแล้วค่าสูงหรือต่ำ สูตร darkness จะกลับให้เองจากสองค่านี้
namespace settings {
const int FLOOR_RAW[5] = {1000, 2000, 3000, 4000, 5000};
const int BLACK_RAW[5] = {900, 900, 900, 900, 900};
const int LINE_THRESHOLD = 450; // Normalized darkness: floor=0, black=1000.

// PWM 0..255 (ตรงกับ L298N เดิมของรถคันนี้: ENA/ENB)
const int BASE_SPEED = 250;     // ความเร็วเดินหน้าปกติ (ค่าจาก BaseSpeed เดิม) — ต้องทดสอบใหม่กับ state machine นี้
constexpr float STEERING_KP = 80; // แก้โค้งธรรมดา; ยังไม่ได้ทดสอบจริงกับรถคันนี้ ปรับทีละ 5
const int TURN_SPEED = 210;     // หมุนสองล้อสวนกันตอนเจอมุม 90/มุมแหลม
const int SEARCH_SPEED = 160;   // หมุนต่อหาเส้นตอนเส้นหายระหว่างเข้าโค้ง
const int REVERSE_SPEED = 160;  // ถอยหาเส้นตอนหลุด
const int CROSS_SPEED = 110;    // เดินหน้าตอนเซนเซอร์เจอดำครบ 5 ตัว (สี่แยก/เส้นตัด)
const int MAX_SPEED = 250;      // เพดาน PWM ของบอร์ดนี้ (0..255)
constexpr float RIGHT_TRIM = 1.0f; // ยังไม่วัด trim ล้อจริงของรถคันนี้ ปรับถ้ารถเบี้ยว

const unsigned long SEARCH_MS = 600;        // หลุดเส้นนานกว่านี้ -> เข้า recovery
const unsigned long TURN_TIMEOUT_MS = 1800; // ติดโค้งแหลมนานเกินนี้ -> เข้า recovery
const unsigned long REVERSE_MS = 650;       // ถอยครบเวลาแล้วรอเส้น เจอเส้นกลับวิ่งเอง
const unsigned long BRAKE_MS = 80;          // เบรกก่อนถอยและก่อนกลับเดินหน้า
const unsigned long ALL_BLACK_MS = 250;     // ดำครบห้าจุดนานเกินนี้ -> ถอยหาเส้น
const unsigned long SAMPLE_MS = 5;          // รอบอ่าน/ควบคุม ไม่มี delay()
const unsigned long REPORT_MS = 100;        // ความถี่ status ผ่าน Serial
const unsigned long CONTROL_GAP_MS = 150;   // อัปเดตช้ากว่านี้ -> STOP กันมอเตอร์ค้าง
}
