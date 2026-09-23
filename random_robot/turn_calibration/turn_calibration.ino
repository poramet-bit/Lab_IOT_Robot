// Measures how long a pivot has to run to cover 90 degrees, so TURN_90_MS in
// obstacle_avoidance_auto.ino can be set from a measurement instead of a guess.
//
// Pins, PWM and the right-wheel trim are copied from obstacle_avoidance_auto so the
// numbers this sketch produces apply to that sketch unchanged. Change TURN_PWM here
// only together with the one there: pivot speed and pivot duration are one setting.
//
// Serial commands (9600 baud, newline ending):
//   r            report the current settings
//   p <pwm>      set the pivot PWM (default 160, same as the avoider)
//   t <ms>       one right pivot of <ms>, reports encoder pulses and estimated degrees
//   l <ms>       same, pivoting left, to check the two directions match
//   a <ms>       four right pivots of <ms> with a pause between, for a 360 degree test
//   m <deg>      the angle YOU measured after the last run; prints the recommended
//                TURN_90_MS computed from it
//   e <deg>      closed-loop turn: pivot until the encoders say <deg>, then brake
//   s <scale>    set TURN_DEG_SCALE used by the estimate and by 'e'
//   1            run the LEFT wheel forward on its own for 600 ms
//   2            run the RIGHT wheel forward on its own for 600 ms
//   f            run both wheels forward for 600 ms
//
// Wiring check, do this first and before any timing work. Send '1': the left wheel
// should roll the robot forward. Send '2': the right wheel should too. A wheel that
// rolls it backward has its IN pair reversed, so set LEFT_INVERTED or RIGHT_INVERTED
// for that wheel here and in obstacle_avoidance_auto. Then 'f' should drive straight
// forward and 't 400' should pivot to the right.
//
// Procedure:
//   1. Put the robot on the floor it actually drives on and mark its heading, for
//      example by lining the chassis up with a floor tile edge or a strip of tape.
//   2. Send 'a 450'. The robot makes four 90 degree pivots. If TURN_90_MS were right
//      it would end up back on the mark.
//   3. Measure the angle it actually covered in total and send it, e.g. 'm 330'.
//   4. Flash the printed TURN_90_MS into obstacle_avoidance_auto.ino and repeat once
//      to confirm; a battery that has drained since the last run changes the answer.
//
// Run this with a freshly charged battery. A timed pivot is open loop, so the same
// milliseconds cover fewer degrees as the pack sags, and a value calibrated on a flat
// battery overshoots on a full one.

constexpr int LEFT_PWM = 5, LEFT_A = 6, LEFT_B = 7;
constexpr int RIGHT_PWM = 10, RIGHT_A = 9, RIGHT_B = 8;
constexpr float RIGHT_TRIM = 46800.0f / 55500.0f;

// Same wiring flags as obstacle_avoidance_auto; keep the two sketches in step or the
// milliseconds measured here describe a pivot the other one never makes.
constexpr bool MOTOR_SIDES_SWAPPED = true;
constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = false;
constexpr int ENC_LEFT = MOTOR_SIDES_SWAPPED ? 3 : 11;
constexpr int ENC_RIGHT = MOTOR_SIDES_SWAPPED ? 11 : 3;
volatile uint16_t encLeftTicks = 0, encRightTicks = 0;

// Chassis geometry, same values as robotcurclerun and spin.
constexpr int DISK_SLOTS = 20;
constexpr float WHEEL_DIAMETER_CM = 6.5f;
constexpr float TRACK_WIDTH_CM = 14.5f;
constexpr float CM_PER_PULSE = 3.14159265f * WHEEL_DIAMETER_CM / DISK_SLOTS;

// Free-spinning wheels slip during a pivot, so the arc the encoders report is longer
// than the arc the chassis actually swept. This is the correction measured on this
// chassis in robotcurclerun (270 degrees commanded came out as 402.5). It only affects
// the printed estimate and the 'e' command; the 'm' recommendation uses your own
// measurement and is independent of it.
float TURN_DEG_SCALE = (360.0f / 270.0f) * (360.0f / 402.5f);

int turnPwm = 160; // matches TURN_PWM in obstacle_avoidance_auto.
unsigned long lastRunMs = 0;   // ms per pivot in the last run
int lastRunPivots = 0;         // how many pivots that run made

constexpr int BRAKE_PWM = 255; // matches obstacle_avoidance_auto; braking duty only.

void setWheel(int pwmPin, int aPin, int bPin, int value) {
  digitalWrite(aPin, value <= 0 ? HIGH : LOW);
  digitalWrite(bPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, value == 0 ? BRAKE_PWM : abs(value)); // A=B=HIGH shorts the motor.
}

void drive(int left, int right) {
  if (LEFT_INVERTED) left = -left;
  if (RIGHT_INVERTED) right = -right;
  int chanA = MOTOR_SIDES_SWAPPED ? right : left;   // pins 5/6/7
  int chanB = MOTOR_SIDES_SWAPPED ? left : right;   // pins 10/9/8
  setWheel(LEFT_PWM, LEFT_A, LEFT_B, chanA);
  setWheel(RIGHT_PWM, RIGHT_A, RIGHT_B, int(chanB * RIGHT_TRIM));
}

void stopMotors() { drive(0, 0); }

void encoderLeftIsr() { encLeftTicks++; }
void encoderRightIsr() { encRightTicks++; }

void resetTicks() {
  noInterrupts();
  encLeftTicks = encRightTicks = 0;
  interrupts();
}

void readTicks(uint16_t &left, uint16_t &right) {
  noInterrupts();
  left = encLeftTicks; right = encRightTicks;
  interrupts();
}

// Both wheels sweep the same arc in a pivot, so the average of the two pulse counts is
// the arc length, and the angle is that arc over the pivot circle.
float degreesFromTicks(uint16_t left, uint16_t right) {
  float arcCm = (left + right) / 2.0f * CM_PER_PULSE;
  return arcCm * 360.0f / (3.14159265f * TRACK_WIDTH_CM) / TURN_DEG_SCALE;
}

void reportRun(const char *label, unsigned long ms, uint16_t left, uint16_t right) {
  Serial.print(label);
  Serial.print(" ms="); Serial.print(ms);
  Serial.print(" pwm="); Serial.print(turnPwm);
  Serial.print(" encL="); Serial.print(left);
  Serial.print(" encR="); Serial.print(right);
  Serial.print(" est_deg="); Serial.println(degreesFromTicks(left, right), 1);
  if (left == 0 || right == 0) {
    Serial.println("WARNING: one encoder read zero. Check ENC_LEFT/ENC_RIGHT wiring; "
                   "the estimate is meaningless until both count.");
  }
}

void pivot(unsigned long ms, bool right) {
  resetTicks();
  if (right) drive(turnPwm, -turnPwm);
  else drive(-turnPwm, turnPwm);
  delay(ms);
  stopMotors();
  delay(400); // let the chassis settle so the next run starts from rest
}

void singlePivot(unsigned long ms, bool right) {
  pivot(ms, right);
  uint16_t l, r;
  readTicks(l, r);
  lastRunMs = ms;
  lastRunPivots = 1;
  reportRun(right ? "pivot right" : "pivot left", ms, l, r);
  Serial.println("Measure the angle it actually turned and send: m <deg>");
}

// Four pivots of the same duration. One 90 degree pivot is hard to eyeball; a full
// circle is not, because the robot either lands back on its mark or it does not.
void fourPivots(unsigned long ms) {
  uint32_t totalLeft = 0, totalRight = 0;
  for (int i = 0; i < 4; i++) {
    pivot(ms, true);
    uint16_t l, r;
    readTicks(l, r);
    totalLeft += l; totalRight += r;
    Serial.print("  pivot "); Serial.print(i + 1);
    Serial.print(": encL="); Serial.print(l);
    Serial.print(" encR="); Serial.print(r);
    Serial.print(" est_deg="); Serial.println(degreesFromTicks(l, r), 1);
    delay(600);
  }
  lastRunMs = ms;
  lastRunPivots = 4;
  Serial.print("4 pivots of "); Serial.print(ms);
  Serial.print("ms: total est_deg=");
  Serial.println(degreesFromTicks(totalLeft / 4, totalRight / 4) * 4.0f, 1);
  Serial.println("Measure the TOTAL angle it covered and send: m <deg>  (360 = perfect)");
}

void recommend(float measuredDeg) {
  if (lastRunPivots == 0) { Serial.println("No run yet. Send 't <ms>' or 'a <ms>' first."); return; }
  if (measuredDeg <= 0.0f) { Serial.println("Measured angle must be > 0."); return; }

  float degPerPivot = measuredDeg / lastRunPivots;
  // Pivot angle is close enough to linear in time over a +-30% window, which is all
  // this correction ever spans; outside that, run the recommended value and measure
  // again rather than trusting one extrapolation.
  float recommended = lastRunMs * 90.0f / degPerPivot;
  Serial.print("measured "); Serial.print(degPerPivot, 1);
  Serial.print(" deg per pivot at "); Serial.print(lastRunMs); Serial.println("ms");
  Serial.print(">>> constexpr unsigned long TURN_90_MS = ");
  Serial.print((unsigned long)(recommended + 0.5f));
  Serial.println(";");
  Serial.print("    (keep RUN_PWM/TURN_PWM at "); Serial.print(turnPwm);
  Serial.println(" in obstacle_avoidance_auto.ino, or this value stops being valid)");
  if (recommended > lastRunMs * 1.3f || recommended < lastRunMs * 0.7f) {
    Serial.println("    That is a big jump. Re-run 'a <new value>' and measure again.");
  }
}

// Closed-loop pivot, for comparison: stops on encoder count rather than on the clock.
// Worth trying because 20 slots per wheel revolution is only about 8 degrees of pivot
// per pulse, so this is coarse, but unlike a timed pivot it does not drift with the
// battery.
void encoderTurn(float deg) {
  float arcCm = (deg * TURN_DEG_SCALE) / 360.0f * 3.14159265f * TRACK_WIDTH_CM;
  long targetPulses = (long)(arcCm / CM_PER_PULSE + 0.5f);
  Serial.print("encoder turn "); Serial.print(deg, 1);
  Serial.print(" deg -> target "); Serial.print(targetPulses); Serial.println(" pulses/wheel");

  resetTicks();
  drive(turnPwm, -turnPwm);
  unsigned long start = millis();
  uint16_t l = 0, r = 0;
  while (true) {
    readTicks(l, r);
    if ((l + r) / 2 >= targetPulses) break;
    if (millis() - start > 3000) { Serial.println("timeout: encoders never reached target"); break; }
  }
  stopMotors();
  delay(400);
  readTicks(l, r);
  reportRun("encoder turn", millis() - start, l, r);
}

void wheelTest(int left, int right, const char *label) {
  Serial.print(label); Serial.println(" forward for 600 ms");
  resetTicks();
  drive(left, right);
  delay(600);
  stopMotors();
  uint16_t l, r;
  readTicks(l, r);
  Serial.print("  encL="); Serial.print(l);
  Serial.print(" encR="); Serial.println(r);
  Serial.println("  Did the robot move FORWARD? If it went backward, invert that wheel.");
}

void report() {
  Serial.print("turnPwm="); Serial.print(turnPwm);
  Serial.print(" TURN_DEG_SCALE="); Serial.print(TURN_DEG_SCALE, 4);
  Serial.print(" CM_PER_PULSE="); Serial.print(CM_PER_PULSE, 4);
  Serial.print(" TRACK_WIDTH_CM="); Serial.println(TRACK_WIDTH_CM, 1);
  Serial.print("pulses for a perfect 90 deg pivot: ");
  Serial.println((90.0f * TURN_DEG_SCALE / 360.0f) * 3.14159265f * TRACK_WIDTH_CM / CM_PER_PULSE, 1);
}

void help() {
  Serial.println("commands: r | p <pwm> | t <ms> | l <ms> | a <ms> | m <deg> | e <deg> | s <scale>");
  Serial.println("wiring check: 1 = left wheel, 2 = right wheel, f = both, all forward");
}

void setup() {
  Serial.begin(9600);
  analogWriteResolution(8);

  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_A, OUTPUT); pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_A, OUTPUT); pinMode(RIGHT_B, OUTPUT);
  stopMotors();

  // Pull-ups before attachInterrupt: it carries an existing pull-up over but cannot
  // add one, and a floating encoder input retriggers off motor noise until the
  // interrupt storm starves the loop and the USB serial with it.
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), encoderLeftIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), encoderRightIsr, RISING);

  Serial.println("turn calibration ready");
  help();
  report();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  float arg = line.length() > 1 ? line.substring(1).toFloat() : 0.0f;

  switch (cmd) {
    case 'r': report(); break;
    case '1': wheelTest(turnPwm, 0, "LEFT wheel"); break;
    case '2': wheelTest(0, turnPwm, "RIGHT wheel"); break;
    case 'f': wheelTest(turnPwm, turnPwm, "BOTH wheels"); break;
    case 'p':
      if (arg < 60 || arg > 255) { Serial.println("pwm out of range (60-255)"); break; }
      turnPwm = (int)arg; report(); break;
    case 't': if (arg > 0) singlePivot((unsigned long)arg, true); else Serial.println("need ms"); break;
    case 'l': if (arg > 0) singlePivot((unsigned long)arg, false); else Serial.println("need ms"); break;
    case 'a': if (arg > 0) fourPivots((unsigned long)arg); else Serial.println("need ms"); break;
    case 'm': recommend(arg); break;
    case 'e': if (arg > 0) encoderTurn(arg); else Serial.println("need deg"); break;
    case 's': if (arg > 0) { TURN_DEG_SCALE = arg; report(); } else Serial.println("need scale"); break;
    default: help(); break;
  }
}
