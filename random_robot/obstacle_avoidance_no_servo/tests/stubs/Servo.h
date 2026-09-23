#pragma once
#include <Arduino.h>

static int fakeServoPin = -1, fakeServoAngle = 90;
static uint32_t fakeServoMovedAt = 0;
struct Servo {
  void attach(int pin) {
    // Motor PWM must already be initialized before Servo reserves a timer.
    assert(pwm[5] == 255 && pwm[10] == 255);
    fakeServoPin = pin;
  }
  void write(int angle) {
    assert(angle >= 0 && angle <= 180);
    assert(pins[6] == HIGH && pins[7] == HIGH && pins[9] == HIGH && pins[8] == HIGH);
    fakeServoAngle = angle;
    fakeServoMovedAt = millis();
  }
};
