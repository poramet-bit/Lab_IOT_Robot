#include <Arduino.h>
#include <Servo.h>

/**
 * ====================================================================
 * โครงการ: Servo 360 Control (0, 90, 180 องศา)
 * รายละเอียด:
 *  - กำหนดขาควบคุม Servo (ค่าเริ่มต้นคือขา A2 ตามโครงสร้างหุ่นยนต์)
 *  - ตั้งค่าเริ่มต้น (Initial Position) ไว้ที่ 90 องศา
 *  - ควบคุมการทำงานให้หมุนเฉพาะค่า: 0, 90, 180 องศา
 *  - ทำงานหมุนวนอัตโนมัติ (0 -> 90 -> 180 -> 90) และรองรับการพิมพ์สั่งผ่าน Serial Monitor
 *
 * หมายเหตุเกี่ยวกับ Servo 360:
 *  1) กรณีเป็น Servo 360 แบบหมุนต่อเนื่อง (Continuous Rotation):
 *     - ค่า 90  : หยุดหมุน (Neutral / STOP)
 *     - ค่า 0   : หมุนทิศทางที่ 1 ด้วยความเร็วเต็มที่ (Full Speed CW/CCW)
 *     - ค่า 180 : หมุนทิศทางที่ 2 ด้วยความเร็วเต็มที่ (Full Speed CCW/CW)
 *     (หากค่า 90 แล้วมอเตอร์ยังขยับช้าๆ สามารถปรับจูน potentiometer ใต้ตัว servo ได้)
 *  2) กรณีเป็น Servo 180/360 แบบกำหนดมุม (Positional Servo):
 *     - ค่า 0   : มุม 0 องศา
 *     - ค่า 90  : มุมกึ่งกลาง 90 องศา
 *     - ค่า 180 : มุม 180 องศา
 * ====================================================================
 */

// กำหนดขาต่อสัญญาณ Servo
const int SERVO_PIN = A2;

// กำหนดระยะเวลาหยุดค้างในแต่ละตำแหน่ง (มิลลิวินาที)
const unsigned long HOLD_TIME_MS = 3000;

Servo myServo;

// ฟังก์ชันสั่งการเซอร์โวและแสดงผลทาง Serial
void setServoAngle(int angle) {
  myServo.write(angle);
  Serial.print("[Servo] ตั้งค่าไปที่: ");
  Serial.print(angle);

  if (angle == 90) {
    Serial.println("° [ตรงกลาง / หยุดหมุน สำหรับแบบ 360 Continuous]");
  } else if (angle == 0) {
    Serial.println("° [มุม 0° / ทิศทาง 1 สำหรับแบบ 360 Continuous]");
  } else if (angle == 180) {
    Serial.println("° [มุม 180° / ทิศทาง 2 สำหรับแบบ 360 Continuous]");
  } else {
    Serial.println("°");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // รอ Serial เชื่อมต่อ (เผื่อกรณีใช้บอร์ด Native USB)
  }

  Serial.println("\n=========================================");
  Serial.println("      Servo 360 Controller (0, 90, 180)  ");
  Serial.println("=========================================");
  Serial.print("Pin ขา Servo: ");
  Serial.println(SERVO_PIN);

  // ทำการเชื่อมต่อขา Servo
  myServo.attach(SERVO_PIN);

  // ตั้งค่าเริ่มต้นไว้ที่ 90 องศาตามที่กำหนด
  Serial.println(">> ตั้งค่าเริ่มต้น (Default Initial Position) เป็น 90 องศา...");
  setServoAngle(90);
  delay(2000); // หน่วงเวลาให้เซอร์โวเข้าตำแหน่งเริ่มต้นให้เรียบร้อย

  Serial.println(">> ระบบพร้อมทำงาน!");
  Serial.println("   - โหมดวนลูป: 0 -> 90 -> 180 -> 90");
  Serial.println("   - สามารถพิมพ์ 0, 90 หรือ 180 ใน Serial Monitor เพื่อสั่งการด้วยตนเองได้");
  Serial.println("-----------------------------------------\n");
}

void loop() {
  // ตรวจสอบคำสั่งผ่าน Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      if (input == "0") {
        Serial.println(">> [คำสั่งจาก Serial]");
        setServoAngle(0);
        delay(HOLD_TIME_MS);
      } else if (input == "90") {
        Serial.println(">> [คำสั่งจาก Serial]");
        setServoAngle(90);
        delay(HOLD_TIME_MS);
      } else if (input == "180") {
        Serial.println(">> [คำสั่งจาก Serial]");
        setServoAngle(180);
        delay(HOLD_TIME_MS);
      } else {
        Serial.print("!! ค่าไม่ถูกต้อง: '");
        Serial.print(input);
        Serial.println("' (อนุญาตเฉพาะค่า: 0, 90, 180 เท่านั้น)");
      }
    }
  }

  // ลำดับการหมุนอัตโนมัติ: 0 -> 90 -> 180 -> 90
  // 1. หมุนไปที่ 0 องศา
  setServoAngle(0);
  delay(HOLD_TIME_MS);

  // 2. หมุนกลับมาที่ 90 องศา
  setServoAngle(90);
  delay(HOLD_TIME_MS);

  // 3. หมุนไปที่ 180 องศา
  setServoAngle(180);
  delay(HOLD_TIME_MS);

  // 4. หมุนกลับมาที่ 90 องศา
  setServoAngle(90);
  delay(HOLD_TIME_MS);
}
