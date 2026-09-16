#pragma once
#include <stdint.h>

struct LineOutput { int left = 0; int right = 0; };

// Pure controller: physical left/right, bit 0 = leftmost sensor.
class LineController {
public:
  enum State { STOPPED, FOLLOWING, SEARCHING };
  State state = STOPPED;
  int basePwm = 35000;
  int searchPwm = 35000;
  int gainPwm = 7000; // PWM correction per sensor position (-2..2).
  int maxPwm = 50000;
  uint32_t searchTimeoutMs = 2000;

  void start() {
    state = FOLLOWING;
    lastSide = 0;
    centerSamples = 0;
  }
  void stop() { state = STOPPED; }

  LineOutput update(uint8_t mask, uint32_t now) {
    LineOutput out;
    if (state == STOPPED) return out;
    const bool center = mask & 4;
    if (state == SEARCHING) {
      if (uint32_t(now - searchStarted) >= searchTimeoutMs) {
        stop();
        return out;
      }
      centerSamples = center ? centerSamples + 1 : 0;
      if (centerSamples < 3) return searchOutput();
      state = FOLLOWING;
      centerSamples = 0;
    }

    // Follow only the contiguous segment containing the center sensor:
    // disconnected outer marks should not pull the robot off a straight line.
    uint8_t selected = mask;
    if (center) {
      selected = 4;
      if (mask & 2) { selected |= 2; if (mask & 1) selected |= 1; }
      if (mask & 8) { selected |= 8; if (mask & 16) selected |= 16; }
    }
    int sum = 0, count = 0;
    for (int i = 0; i < 5; ++i) {
      if (selected & (1 << i)) { sum += i - 2; ++count; }
    }
    // Record an unambiguous side; never overwrite it with an empty reading.
    if (count && sum != 0) lastSide = sum < 0 ? -1 : 1;
    const bool leftCorner = !center && (mask & 1) && !(mask & 24);
    const bool rightCorner = !center && (mask & 16) && !(mask & 3);
    if (!count || leftCorner || rightCorner) {
      if (!lastSide) { stop(); return out; }
      state = SEARCHING;
      searchStarted = now;
      centerSamples = 0;
      return searchOutput();
    }
    const int correction = gainPwm * sum / count;
    out.left = limited(basePwm + correction);
    out.right = limited(basePwm - correction);
    return out;
  }

private:
  int lastSide = 0;
  int centerSamples = 0;
  uint32_t searchStarted = 0;
  int limited(int pwm) const {
    if (pwm < 0) return 0;
    return pwm > maxPwm ? maxPwm : pwm;
  }
  LineOutput searchOutput() const {
    LineOutput out;
    // Forward pivot with the inner wheel braked; no reverse on the fixed caster.
    if (lastSide < 0) out.right = limited(searchPwm);
    else out.left = limited(searchPwm);
    return out;
  }
};
