#include <Servo.h>

#define SERVO_PIN A2
#define MODE_STANDARD 0   // 0-180 deg servo
#define MODE_360 1        // continuous rotation servo
#define MODE MODE_STANDARD

Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
}

void loop() {
  if (MODE == MODE_STANDARD) {
    myServo.write(0);
    Serial.println("angle: 0");
    delay(1000);

    myServo.write(90);
    Serial.println("angle: 90");
    delay(1000);

    myServo.write(180);
    Serial.println("angle: 180");
    delay(1000);
  } else {
    myServo.write(0);   // full speed CCW
    Serial.println("360: CCW full speed");
    delay(1500);

    myServo.write(90);  // stop
    Serial.println("360: stop");
    delay(1500);

    myServo.write(180); // full speed CW
    Serial.println("360: CW full speed");
    delay(1500);
  }
}
