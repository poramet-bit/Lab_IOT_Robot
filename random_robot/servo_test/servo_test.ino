#include <Arduino.h>
#include <Servo.h>

// กำหนดขา Servo ตามโครงสร้างหุ่นยนต์ (ขา A2)
const int SERVO_PIN = A2;

Servo myServo;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // รอ Serial เชื่อมต่อ (เผื่อบอร์ด UNO R4 / Native USB)
  }

  Serial.println("=================================");
  Serial.println("   Servo Test (0 - 180 deg)      ");
  Serial.println("=================================");
  Serial.print("Attaching Servo to Pin: ");
  Serial.println(SERVO_PIN);

  myServo.attach(SERVO_PIN);
  
  // ตั้งค่าเริ่มต้นไปที่ 90 องศา (ตรงกลาง) สั้นๆ ก่อนเริ่มลูป
  myServo.write(90);
  delay(1000);
}

void loop() {
  // หมุนไปที่ 0 องศา
  Serial.println("[Servo] Moving to 0 degrees...");
  myServo.write(0);
  delay(5000); // รอ 5 วินาที

  // หมุนไปที่ 180 องศา
  Serial.println("[Servo] Moving to 180 degrees...");
  myServo.write(180);
  delay(5000); // รอ 5 วินาที
}
