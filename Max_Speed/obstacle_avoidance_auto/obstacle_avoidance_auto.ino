#include <Servo.h>

// Chassis, measured: 30 cm long, 20 cm wide, 9 cm tall (14 cm to the top of the sonar).
// Wheels 65 mm across on a 145 mm track, axle halfway down the body, 1:48 gearboxes.
// That works out at roughly 32 cm/s at RUN_PWM and a 11.4 cm arc per wheel for a 90
// degree pivot. Every distance and duration derived from those numbers says so where
// it is defined, so re-derive them rather than nudging them if the chassis changes.

// Motor wiring/direction from IOT-Robot/Robot/robotline_fullspeed.
constexpr int LEFT_PWM = 5, LEFT_A = 6, LEFT_B = 7;
constexpr int RIGHT_PWM = 10, RIGHT_A = 9, RIGHT_B = 8;
constexpr float RIGHT_TRIM = 46800.0f / 55500.0f;

// Wiring confirmed on the robot: ENA=5 IN1=6 IN2=7 drive the LEFT wheel, IN3=8 IN4=9
// ENB=10 drive the RIGHT wheel. That is what this sketch already assumed, so the sides
// are not swapped.
constexpr bool MOTOR_SIDES_SWAPPED = true;
// One flag per wheel, because the fault turned out to be one motor's IN pair wired
// backwards rather than both. With a single wheel reversed, a forward command spins the
// robot on the spot and a pivot command drives it in a straight line, which is why it
// reversed a long way and never left the start.
//
// Find the right values with turn_calibration: '1' runs the left wheel forward on its
// own and '2' the right. Whichever wheel rolls the robot backwards is the inverted one.
constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = false;

// The other sketches in this folder disagree about which of these two pins is which:
// obstacle_avoidance and servo_ir_range_test say TRIG=A1 ECHO=A0, ultrasonic_check
// says the opposite. Wired backwards the sensor never answers and the robot reads open
// floor everywhere, so rather than trust either, setup() tries both orders and keeps
// whichever one echoes. These are variables because that probe assigns them.
int trigPin = A1, echoPin = A0;
constexpr int SERVO_PIN = A2;
// If the servo horn is mounted mirrored, 180 points right instead of left and every
// side decision comes out backwards. Run servo_ir_range_test.ino to confirm, then flip
// SERVO_MIRRORED instead of editing the angles.
constexpr bool SERVO_MIRRORED = false;
constexpr int SERVO_FRONT = 90;
// 60 degrees off centre, not 90. Swung all the way to 0 or 180 the sensor ends up
// looking along the chassis and hears the robot's own body and wiring: the side scans
// were coming back as 2-5 cm on both sides at once, which is not a reading any real
// pair of obstacles can produce, and the robot then believed it was walled in every
// single time. 60 degrees still clears the chassis and is close enough to sideways to
// answer the only question being asked, which is whether there is room to turn.
constexpr int SERVO_SIDE_ANGLE = 60;
constexpr int SERVO_LEFT = SERVO_MIRRORED ? SERVO_FRONT - SERVO_SIDE_ANGLE
                                          : SERVO_FRONT + SERVO_SIDE_ANGLE;
constexpr int SERVO_RIGHT = SERVO_MIRRORED ? SERVO_FRONT + SERVO_SIDE_ANGLE
                                           : SERVO_FRONT - SERVO_SIDE_ANGLE;
// Anything this close to a sideways-pointing sensor is the robot itself, not an
// obstacle: the chassis is 20 cm wide and the sonar sits on top of it. Such a reading
// carries no information about the room, so it must not count as a wall -- treating it
// as one is what deadlocked the robot into turning around over and over.
constexpr float SIDE_BLIND_CM = 8.0f;
// A 90 degree sweep needs ~250-350 ms on a hobby servo; a shorter glance samples
// while the horn is still moving and reports the wrong side's distance.
constexpr uint32_t SERVO_SETTLE_MS = 300, SERVO_GLANCE_MS = 280;
// Back to 30 ms after 12 ms stopped the sensor answering at all. pulseIn spends this
// budget waiting for the echo to BEGIN, not just to come back, and a clone module can
// take most of it before it starts its pulse; cut the budget and every reading times
// out, which this sketch reads as open floor. Lower it only with the raw echo_us
// figure from the debug line in hand.
constexpr unsigned long ECHO_TIMEOUT_US = 30000;
// An HC-SR04 keeps ringing for tens of milliseconds after a burst. Fire the next ping
// too soon and it hears the tail of the previous one, which comes back as a long
// reading, which reads as open floor in front of a wall the robot is staring at.
constexpr unsigned long PING_GAP_MS = 60;

// The only IR sensors on this robot are the two at the front.
// Measured on this robot: the sensor on A3 sits on the right of the chassis and the
// one on A4 on the left, the same mirroring the motor wiring has. Getting this wrong
// steers the robot into the obstacle the IR just found, since irSide decides which
// way not to turn.
constexpr bool IR_SIDES_SWAPPED = true;
constexpr int IR_FRONT_LEFT = IR_SIDES_SWAPPED ? A4 : A3;
constexpr int IR_FRONT_RIGHT = IR_SIDES_SWAPPED ? A3 : A4;
// This robot carries no rear sensors, so nothing watches the ground it reverses over.
// The backup is kept short and is the only blind movement in the sketch.
constexpr bool IR_ACTIVE_LOW = true; // verify with a Serial print; invert if opposite.

// LM393 slotted encoders, one per wheel, wired the same way as robotcurclerun and
// simple_avoid: the disc on the D5 (left) motor reads on D11, the disc on the D10
// (right) motor reads on D3. Having these two swapped inverts the sign of the
// straight-line error and the controller then steers the robot further off course
// instead of back onto it.
// Both pins are interrupt capable on the UNO R4 WiFi (ICU channels 4 and 0), so a
// plain attachInterrupt covers both wheels.
constexpr int ENC_LEFT = MOTOR_SIDES_SWAPPED ? 3 : 11;
constexpr int ENC_RIGHT = MOTOR_SIDES_SWAPPED ? 11 : 3;
volatile uint16_t encLeftTicks = 0, encRightTicks = 0;

// Tune these on the real robot.
// TURN_PWM sits below RUN_PWM: a pivot that snaps round is hard to watch and overshoots
// its angle, and unlike driving it does not need the speed. Do not take it much under
// 140 -- a pivot fights both wheels' static friction at once and a lazy one stalls
// halfway through the angle.
constexpr int RUN_PWM = 180, TURN_PWM = 160, REVERSE_PWM = 130;
// Backing off before a pivot, measured rather than timed. 65 mm wheels on 20-slot
// discs give 1.02 cm of travel per encoder pulse, so 10 cm is 10 pulses per wheel and
// stays 10 cm whatever the battery is doing -- a timed reverse covers less ground as
// the pack sags. The move is blind, since the robot has no rear sensor, which is why
// it is bounded by distance and by a timeout rather than by a duration alone.
constexpr float REVERSE_CM = 10.0f;
constexpr float CM_PER_PULSE = 3.14159265f * 6.5f / 20.0f;
constexpr unsigned long REVERSE_TIMEOUT_MS = 900;
// FRONT_DETECT_CM is the sighting distance: brake there, then scan and decide standing
// still. There is no second, closer threshold any more, because there is nothing the
// robot could do differently -- with no rear sensor it never reverses, so every
// obstacle is answered by pivoting on the spot whatever the distance.
// 25 cm leaves little room: at RUN_PWM the robot covers a few centimetres between one
// ping and the next, and the brake needs its own distance on top. It works because the
// reverse kick in brakeHard() stops the chassis rather than letting it coast, but if
// the robot starts nudging obstacles, lower RUN_PWM before raising this back.
// Derived from the chassis, not picked by feel. The robot measures 30 x 20 cm, so a
// pivot on the spot sweeps a circle of sqrt(15^2 + 10^2) = 18.0 cm around its centre
// while the sonar sits about 15 cm ahead of that centre: the rear corners swing out
// 3 cm past where the sonar was pointing. Add one ping interval of blind travel
// (about 3 cm at RUN_PWM) and the braking distance (about 5 cm) and 20 leaves 9 cm of
// margin. Re-derive this if RUN_PWM, PING_GAP_MS or the chassis changes; do not just
// nudge it. The 15 cm assumes the wheel axle sits halfway down the body -- if it is
// further back, the sonar is further ahead of the pivot and this number has to grow.
// The geometry above sets the floor at 20, but the sonar's own readings wander by a
// couple of centimetres between pings, and at 20 the robot drove straight past
// obstacles that were measuring 21 and 22. 30 covers that spread and still leaves the
// pivot its clearance.
constexpr float FRONT_DETECT_CM = 30.0f, SIDE_MIN_CM = 25.0f;
// Motor brake is not instant; let the chassis settle before the sonar sweep, or the
// robot is still coasting while it measures the sides.
constexpr unsigned long BRAKE_SETTLE_MS = 250;
// Pivot angle is duration times pivot speed, so raising TURN_PWM from 180 to 220
// shortened the time a 90 degree pivot needs. 330 is that ratio applied to the old
// 400 and nothing more: it is an estimate, not a measurement. Run turn_calibration
// ('a 330', then 'm <measured degrees>') and put its answer here.
// Scaled from the 220 PWM figures by 220/160, since a slower pivot needs longer to
// cover the same angle. Still arithmetic rather than measurement: run turn_calibration.
constexpr unsigned long TURN_90_MS = 450;
// Not TURN_90_MS * 2. Every pivot spends a fixed slice at the start breaking the
// wheels out of static friction and only then turns at speed, so a half turn driven
// for twice the quarter-turn time comes up short -- measured at about 150 of the 180
// degrees asked for. 790 is 660 scaled by 180/150. Measure it: turn_calibration
// 't 790' turns once, then 'm <degrees you measured>' prints the value to use.
constexpr unsigned long TURN_180_MS = 1090;

// Straight-line speed control. The wheels never match at the same PWM, so the tick
// counts are compared every CONTROL_MS and the difference is split between them:
// the fast wheel gives up as much PWM as the slow one gains, keeping average speed.
constexpr unsigned long CONTROL_MS = 50;
constexpr int START_PWM = 120;   // enough to break static friction from a standstill
constexpr int RAMP_STEP = 8;     // PWM gained per control tick while ramping up
constexpr float STRAIGHT_KP = 4.0f;  // PWM per tick of error, reacts to the deviation
constexpr float STRAIGHT_KI = 1.0f;  // accumulates, cancels a constant motor mismatch
constexpr int MAX_CORRECTION = 60;
// A dead encoder reads zero forever, which the controller would read as a fully
// stalled wheel and answer with maximum correction, spinning the robot in place.
constexpr int ENC_SILENT_TICKS_FAULT = 10;

int straightPwm = 0;
float straightIntegral = 0.0f;
unsigned long lastControlMs = 0;
int encSilentLeft = 0, encSilentRight = 0;
bool encoderFault = false;

// The IR is ignored for this long at the start of an escape burst, and no longer.
// The obstacle the robot just backed away from is still inside IR range, so honouring
// the IR immediately would end the burst on the spot and the stop-reverse-turn cycle
// would repeat forever. After the grace window the IR is live again, so anything new
// in front brakes the robot at once.
constexpr unsigned long ESCAPE_IR_GRACE_MS = 250;

// After a maneuver, commit to a short forward run. A front IR sees far wider than
// the backup distance, so without this burst it still sees the obstacle it just
// backed away from, re-triggers on the next loop, and the robot cycles
// stop-reverse-turn forever without ever driving off.
constexpr unsigned long ESCAPE_FORWARD_MS = 700;
// Give up on side-stepping and turn around after this many maneuvers in a row.
constexpr int MAX_CONSECUTIVE_AVOIDS = 3;

// Short-term memory of where obstacles were found, so a heading already known to be
// blocked is not re-tried. Entries decay: the robot keeps moving and a blocked
// heading goes stale within a few seconds.
constexpr unsigned long MEMORY_MS = 5000;
constexpr int DIR_LEFT = 0, DIR_FRONT = 1, DIR_RIGHT = 2, DIR_COUNT = 3;
unsigned long blockedAt[DIR_COUNT] = {0, 0, 0};
int consecutiveAvoids = 0;
bool firstSighting = true;

// Where the robot has already driven. The blocked memory above is robot-relative and
// only remembers walls; this one is world-relative and remembers ground already
// covered, which is what stops the robot from turning back into the corridor it just
// came out of. Heading is a quarter turn counter, 0-3, updated by every turn; the
// robot has no compass, so it is dead reckoning and drifts, but it stays good enough
// over the few seconds that matter here.
constexpr unsigned long VISIT_MEMORY_MS = 15000;
int heading = 0;
unsigned long headingVisitedAt[4] = {0, 0, 0, 0};
// Which side the last turn went to, so the next free choice can go the other way.
bool lastTurnRight = false;
// Sonar on a rough floor is worth a couple of centimetres either way, so only call a
// side roomier when it wins by more than this. Inside the margin the two sides count
// as equal and the tie-break decides.
constexpr float SIDE_ADVANTAGE_CM = 15.0f;

// How a free choice between two clear sides is made.
//   POLICY_RANDOM     a coin flip
//   POLICY_ROOMIER    the side the sonar says is clearer; a tie goes right
//   POLICY_ALTERNATE  swap sides every turn: left, right, left, ...
//   POLICY_LEFT       always left
//   POLICY_RIGHT      always right
//   POLICY_SMART      unexplored heading first, then the roomier side
constexpr int POLICY_RANDOM = 0, POLICY_ROOMIER = 1, POLICY_ALTERNATE = 2,
              POLICY_LEFT = 3, POLICY_RIGHT = 4, POLICY_SMART = 5;
// SMART rather than RANDOM: a random walk keeps re-entering ground it just left, and
// in a closed box that shows up as the robot orbiting one area instead of finding the
// gap. SMART prefers a heading it has not driven in the last VISIT_MEMORY_MS, which
// pushes it into new ground and towards the walls it has not inspected yet. It is
// still not a search -- it has no idea where the gap is -- but it stops covering the
// same few square metres over and over.
constexpr int TURN_POLICY = POLICY_SMART;

// true: sweep the servo left and right before turning, so the robot knows which side
// is actually open. It costs two servo settles plus a brake settle, close to a second
// standing still, but without it the robot is blind sideways and turns into walls it
// never saw.
constexpr bool SCAN_BEFORE_TURN = true;
// Unmeasured sides report this, which is past every threshold and so reads as clear.
constexpr float SIDE_UNKNOWN_CM = 400.0f;
// Far enough ahead that there is nothing to decide. A ping that finds nothing at all
// times out and reports SIDE_UNKNOWN_CM, which is also past this, so both "wide open"
// and "no echo anywhere" count as open road. Well under the sensor's usable range, so
// it is a real measurement and not the edge of what the module can hear.
constexpr float FRONT_OPEN_CM = 150.0f;

// The arena is a closed box roughly 3 m on a side, so from anywhere inside it every
// heading has a wall within about 300 cm. A reading past this, or a ping that hears
// nothing at all and so reports SIDE_UNKNOWN_CM, means the sonar is looking through
// the gap in the wall rather than at a wall: that is the way out. This is why the
// number is a property of the room, not of the robot -- measure the room and set it,
// with enough margin over the diagonal that a long but legitimate indoor reading
// cannot reach it.
constexpr float EXIT_OPEN_CM = 320.0f;
// There is no separate, longer run for heading out of the gap. Every forward burst in
// this sketch is ESCAPE_FORWARD_MS, because the burst is not what carries the robot
// anywhere: when it ends, loop() goes straight back to ordinary driving and keeps
// going as long as the way is clear. A second run length only made the robot's
// behaviour harder to read from the outside.
bool exitFound = false;

// While the way ahead is clear the sonar stares straight forward, which is how the
// robot drove along the back wall and straight past the gap in it without ever looking
// at it. Every PATROL_GLANCE_MS of clear driving it stops and glances at one side,
// alternating, purely to catch an opening it is passing. A glance costs one servo
// settle out and one back, so keep the interval well above that or the robot spends
// more time looking than driving.
constexpr unsigned long PATROL_GLANCE_MS = 2500;
unsigned long lastGlanceMs = 0;
bool glanceRightNext = false;

void markHeadingVisited() { headingVisitedAt[heading & 3] = millis(); }

bool recentlyVisited(int h) {
  h &= 3;
  return headingVisitedAt[h] != 0 && millis() - headingVisitedAt[h] < VISIT_MEMORY_MS;
}

Servo sonar;

void markBlocked(int dir) { blockedAt[dir] = millis(); }
void markClear(int dir) { blockedAt[dir] = 0; }

bool recentlyBlocked(int dir) {
  return blockedAt[dir] != 0 && millis() - blockedAt[dir] < MEMORY_MS;
}

// The memory is robot-relative, so turning has to rotate it. Turning right moves
// what was ahead to the left, and what was on the right to the front; the new right
// used to be behind us and was never measured, so it resets to unknown.
void rotateMemoryRight() {
  unsigned long front = blockedAt[DIR_FRONT], right = blockedAt[DIR_RIGHT];
  blockedAt[DIR_LEFT] = front;
  blockedAt[DIR_FRONT] = right;
  blockedAt[DIR_RIGHT] = 0;
}

void rotateMemoryLeft() {
  unsigned long front = blockedAt[DIR_FRONT], left = blockedAt[DIR_LEFT];
  blockedAt[DIR_RIGHT] = front;
  blockedAt[DIR_FRONT] = left;
  blockedAt[DIR_LEFT] = 0;
}

// A half turn swaps the sides and puts unmeasured ground ahead.
void rotateMemoryAround() {
  unsigned long left = blockedAt[DIR_LEFT];
  blockedAt[DIR_LEFT] = blockedAt[DIR_RIGHT];
  blockedAt[DIR_RIGHT] = left;
  blockedAt[DIR_FRONT] = 0;
}

// A=B=HIGH shorts the motor and brakes; the PWM sets how hard. Full, because an
// obstacle sighting has to end in a stop, not a slowdown.
constexpr int BRAKE_PWM = 255;
// Shorting the motor only removes drive; the chassis still carries its momentum into
// the obstacle. A brief burst of reverse drive cancels that momentum first, the same
// counter-torque trick robotcurclerun uses to stop a turn overshooting. Too long and
// the burst stops braking and starts driving the robot backwards.
constexpr int BRAKE_KICK_PWM = 150;
constexpr unsigned long BRAKE_KICK_MS = 45;
// Only pulse against real motion. Without this, holding an obstacle in front of the
// IR would fire a reverse burst every pass of loop() and walk the standing robot
// backwards.
bool motorsMoving = false;

void setWheel(int pwmPin, int aPin, int bPin, int value) {
  digitalWrite(aPin, value <= 0 ? HIGH : LOW);
  digitalWrite(bPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, value == 0 ? BRAKE_PWM : abs(value));
}

void drive(int left, int right) {
  if (LEFT_INVERTED) left = -left;
  if (RIGHT_INVERTED) right = -right;
  int chanA = MOTOR_SIDES_SWAPPED ? right : left;   // pins 5/6/7
  int chanB = MOTOR_SIDES_SWAPPED ? left : right;   // pins 10/9/8
  setWheel(LEFT_PWM, LEFT_A, LEFT_B, chanA);
  setWheel(RIGHT_PWM, RIGHT_A, RIGHT_B, int(chanB * RIGHT_TRIM));
  motorsMoving = left != 0 || right != 0;
}

// Any stop ends the current straight run, so the next one ramps up from rest again
// instead of slamming in at cruise PWM with a stale correction.
void resetStraight() {
  straightPwm = 0;
  straightIntegral = 0.0f;
  encSilentLeft = encSilentRight = 0;
  noInterrupts();
  encLeftTicks = encRightTicks = 0;
  interrupts();
  // Backdate the tick so the next driveStraight() call acts at once instead of
  // leaving the robot sitting still for one control period.
  lastControlMs = millis() - CONTROL_MS;
}

void stopMotors() {
  drive(0, 0);
  resetStraight();
}

// Stop now, not eventually: reverse drive kills the momentum, then the short brake
// holds the wheels. Use this wherever something has been sighted; plain stopMotors()
// is for ending a maneuver that has already run its course.
void brakeHard() {
  if (!motorsMoving) { stopMotors(); return; }
  drive(-BRAKE_KICK_PWM, -BRAKE_KICK_PWM);
  delay(BRAKE_KICK_MS);
  stopMotors();
}

// Reverses until both wheels have covered REVERSE_CM, or until the timeout if an
// encoder is silent. Without the timeout a dead encoder would mean reversing forever.
void reverseCm(float cm) {
  uint16_t target = (uint16_t)(cm / CM_PER_PULSE + 0.5f);
  noInterrupts();
  encLeftTicks = encRightTicks = 0;
  interrupts();

  drive(-REVERSE_PWM, -REVERSE_PWM);
  unsigned long start = millis();
  uint16_t left = 0, right = 0;
  while (millis() - start < REVERSE_TIMEOUT_MS) {
    noInterrupts();
    left = encLeftTicks; right = encRightTicks;
    interrupts();
    if ((left + right) / 2 >= target) break;
  }
  stopMotors();
  Serial.print("reversed "); Serial.print((left + right) / 2 * CM_PER_PULSE, 1);
  Serial.print("cm of "); Serial.print(cm, 0); Serial.println("cm");
}

void encoderLeftIsr() { encLeftTicks++; }
void encoderRightIsr() { encRightTicks++; }

// One edge per slot, matching the other sketches on this chassis. CHANGE would count
// both edges, doubling the noise a dirty slot or a motor spike gets to inject.
// attachInterrupt() rewrites the pin configuration and only keeps a pull-up that
// pinMode() already enabled, so the INPUT_PULLUP calls in setup() must run first: a
// floating encoder input retriggers continuously off motor noise, and that interrupt
// storm starves the loop and the R4's USB CDC serial, which shows up as the board
// freezing and its port vanishing from the IDE.
void attachEncoders() {
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), encoderLeftIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), encoderRightIsr, RISING);
}

// Call repeatedly while driving forward: ramps to cruise speed and keeps the wheel
// tick rates equal so the robot tracks straight.
void driveStraight() {
  if (millis() - lastControlMs < CONTROL_MS) return;
  lastControlMs = millis();

  uint16_t left, right;
  noInterrupts();
  left = encLeftTicks; right = encRightTicks;
  encLeftTicks = encRightTicks = 0;
  interrupts();

  if (straightPwm == 0) straightPwm = START_PWM;
  else if (straightPwm < RUN_PWM) straightPwm = min(straightPwm + RAMP_STEP, RUN_PWM);

  // Only judge an encoder silent once there is enough PWM that the wheel must turn.
  if (straightPwm >= START_PWM && !encoderFault) {
    encSilentLeft = left == 0 ? encSilentLeft + 1 : 0;
    encSilentRight = right == 0 ? encSilentRight + 1 : 0;
    if (encSilentLeft >= ENC_SILENT_TICKS_FAULT || encSilentRight >= ENC_SILENT_TICKS_FAULT) {
      encoderFault = true;
      Serial.println("encoder silent -> closed loop off, running open loop");
    }
  }

  if (encoderFault) {
    drive(straightPwm, straightPwm); // RIGHT_TRIM alone, same as before the encoders.
    return;
  }

  int error = (int)left - (int)right; // positive: left wheel is running faster
  straightIntegral += STRAIGHT_KI * error;
  straightIntegral = constrain(straightIntegral, -MAX_CORRECTION, MAX_CORRECTION);

  int correction = constrain((int)(STRAIGHT_KP * error + straightIntegral),
                             -MAX_CORRECTION, MAX_CORRECTION);

  int leftPwm = constrain(straightPwm - correction / 2, 0, 255);
  int rightPwm = constrain(straightPwm + correction / 2, 0, 255);
  drive(leftPwm, rightPwm);
}

bool irTriggered(int pin) {
  return digitalRead(pin) == (IR_ACTIVE_LOW ? LOW : HIGH);
}

// true parks the robot when the startup self test finds no echo at all. Blind driving
// is how it hit the first wall at full speed, but a working robot that will not move
// is its own kind of broken, so this is a switch rather than a rule.
constexpr bool SONAR_REQUIRED = false;
bool sonarDead = false;
void applySonarPins(int trig, int echo) {
  trigPin = trig; echoPin = echo;
  pinMode(trigPin, OUTPUT); digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  delay(10); // let the module see a settled trigger line before the next burst
}

unsigned long lastPingMs = 0;
float lastFrontCm = 400.0f;
unsigned long lastEchoUs = 0; // raw pulse width, 0 means the sensor never answered

float pingCm() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  lastPingMs = millis();
  lastEchoUs = duration;
  lastFrontCm = duration == 0 ? 400.0f : duration * 0.0343f / 2.0f;
  return lastFrontCm;
}

// What the front sensor currently says, pinging only when the sensor has had its
// PING_GAP_MS to go quiet. Called flat out in a loop, pingCm() fires far faster than
// that and each burst hears the tail of the one before it, which comes back long: the
// robot drove at walls because its own echoes kept reporting open floor. Rationing
// the pings is what makes them true, and between them this returns the last reading,
// so the caller never blocks and the IR stays sampled every pass.
float frontDistance() {
  if (millis() - lastPingMs >= PING_GAP_MS) return pingCm();
  return lastFrontCm;
}

// Closest of three spaced pings. Only used once the robot is already stopped: braking
// comes first, confirming second, so a bad echo costs a brief pause and never a missed
// obstacle. The closest reading wins rather than the median, because the ways this
// sensor fails all read long: a burst that glances off an angled surface, a ping that
// catches the previous one's ringing, and a timeout all report open floor, while
// nothing makes it invent an obstacle that is not there. One sighting out of three is
// therefore worth more than two clear readings.
float pingCmConfirm() {
  float closest = pingCm();
  for (int i = 0; i < 2; i++) {
    delay(PING_GAP_MS);
    closest = min(closest, pingCm());
  }
  return closest;
}

// The servo settle is long enough that the previous ping has died out, but two pings
// taken at the same angle still need the gap between them. Two are taken because this
// reading decides which side the robot commits to.
float lookAt(int angle, uint32_t settleMs = SERVO_SETTLE_MS) {
  sonar.write(angle);
  delay(settleMs);
  float first = pingCm();
  delay(PING_GAP_MS);
  return min(first, pingCm());
}

void turnLeft(unsigned long ms) {
  drive(-TURN_PWM, TURN_PWM); delay(ms); stopMotors();
  rotateMemoryLeft();
  heading = (heading + 3) & 3;
  lastTurnRight = false;
}

void turnRight(unsigned long ms) {
  drive(TURN_PWM, -TURN_PWM); delay(ms); stopMotors();
  rotateMemoryRight();
  heading = (heading + 1) & 3;
  lastTurnRight = true;
}

// A half turn ends up in the same place whichever way it spins, but the spin still
// counts as a turn: it swaps sides like any other so the following free choice does
// not land on the side this one just swept through.
void turnAround() {
  bool spinRight = !lastTurnRight;
  if (spinRight) drive(TURN_PWM, -TURN_PWM);
  else drive(-TURN_PWM, TURN_PWM);
  delay(TURN_180_MS); stopMotors();
  rotateMemoryAround();
  heading = (heading + 2) & 3;
  lastTurnRight = spinRight;
}

// Drive forward past the obstacle the IR was still staring at when the maneuver
// finished. The sonar has a veto the whole way and the IR regains its veto once the
// grace window is over, so the burst cannot drive the robot into anything.
void escapeForward() {
  unsigned long start = millis();
  while (millis() - start < ESCAPE_FORWARD_MS) {
    driveStraight();
    markHeadingVisited();

    // Brake first, ask afterwards. pingCm() blocks for up to ECHO_TIMEOUT_US, so the
    // next check is already 30 ms of travel away; there is no spare time to spend on
    // confirming a reading before cutting the motors.
    if (frontDistance() < FRONT_DETECT_CM) { brakeHard(); return; }

    if (millis() - start >= ESCAPE_IR_GRACE_MS &&
        (irTriggered(IR_FRONT_LEFT) || irTriggered(IR_FRONT_RIGHT))) {
      brakeHard();
      Serial.println("IR during escape -> brake");
      return;
    }
  }
}

// Last resort when short side-steps keep failing: sweep the sonar across all three
// headings and commit to whichever one measures farthest, rather than turning around
// on principle. Turning around was a guess that the way back is open, and in a corner
// or a doorway it often is not; the sweep answers the question instead of assuming it.
//
// Distances are compared against each other, not against a threshold, so the widest
// gap wins even when everything is tight. Only if nothing anywhere beats SIDE_MIN_CM
// does the robot fall back on the half turn.
//
// irSide: -1 left IR tripped, +1 right IR tripped, 0 neither. A tripped IR vetoes its
// side whatever the sonar reports, because the IR sees a few centimetres ahead while
// the sonar cone can look straight over a low obstacle.
void escapeToFarthest(int irSide) {
  float front = lookAt(SERVO_FRONT);

  // Nothing ahead within FRONT_OPEN_CM: no side can beat that, so skip the two side
  // sweeps and their servo settles and just go. Only the IR can object, and it sees
  // closer than the sonar does. Past EXIT_OPEN_CM it is not just clear, it is outside
  // the arena, so commit to the longer run that carries the robot through the gap.
  if (irSide == 0 && front >= FRONT_OPEN_CM) {
    bool isExit = front >= EXIT_OPEN_CM && confirmExit(SERVO_FRONT);
    if (isExit) exitFound = true;
    Serial.print("sweep F="); Serial.print(front, 0);
    Serial.println(isExit ? " -> EXIT ahead, driving out" : " -> nothing ahead, driving straight on");
    escapeForward();
    return;
  }

  float right = lookAt(SERVO_RIGHT);
  float left = lookAt(SERVO_LEFT);
  sonar.write(SERVO_FRONT);
  delay(SERVO_SETTLE_MS);

  // A vetoed heading is scored below every real reading so it can never win. Readings
  // from inside the robot's own footprint are vetoed the same way: they are noise, and
  // letting them compete would send the robot at whichever side it misread.
  if (left < SIDE_BLIND_CM) left = -1.0f;
  if (right < SIDE_BLIND_CM) right = -1.0f;
  if (irSide < 0) left = -1.0f;
  if (irSide > 0) right = -1.0f;
  if (irSide != 0) front = -1.0f; // an IR hit is in front of the robot by definition
  // Front only counts as an option if driving at it would not trip the brake on the
  // first pass. Winning the comparison at 22 cm is not the same as being open: the
  // escape burst would stop dead and the robot would be straight back here.
  if (front < FRONT_DETECT_CM) front = -1.0f;

  Serial.print("sweep F="); Serial.print(front, 0);
  Serial.print(" L="); Serial.print(left, 0);
  Serial.print(" R="); Serial.print(right, 0);
  Serial.print(" irSide="); Serial.println(irSide);

  float best = max(front, max(left, right));

  if (best < SIDE_MIN_CM) {
    Serial.println("nothing open anywhere -> turning around");
    turnAround();
    escapeForward();
    return;
  }

  bool isExit = best >= EXIT_OPEN_CM &&
                confirmExit(best == front ? SERVO_FRONT
                                          : (best == right ? SERVO_RIGHT : SERVO_LEFT));
  if (isExit) {
    exitFound = true;
    Serial.println("EXIT: widest gap reads past the arena -> heading out");
  }

  if (best == front) {
    Serial.println("front is the widest gap and clear -> driving straight on");
  } else if (best == right) {
    Serial.println("right is the widest gap");
    turnRight(TURN_90_MS);
  } else {
    Serial.println("left is the widest gap");
    turnLeft(TURN_90_MS);
  }
  escapeForward();
}

// A single long reading is not enough to commit to driving out of the arena. Boxes are
// placed at random inside it, and a sonar burst that meets a box edge or any surface at
// a shallow angle scatters away instead of coming back, which reports the same
// "nothing there" as an actual opening. A real gap answers the same way three times;
// a glancing reflection usually does not survive the servo settling and re-aiming.
bool confirmExit(int angle) {
  sonar.write(angle);
  delay(SERVO_SETTLE_MS);
  bool open = true;
  for (int i = 0; i < 3 && open; i++) {
    if (i) delay(PING_GAP_MS);
    if (pingCm() < EXIT_OPEN_CM) open = false;
  }
  sonar.write(SERVO_FRONT);
  delay(SERVO_SETTLE_MS);
  return open;
}

// Looks at one side mid-run, to catch an opening the robot is driving past rather than
// driving at. Only an opening acts on it: anything else it sees is left to the front
// sensors, because a side reading taken in passing says nothing about whether the way
// ahead is clear.
void patrolGlance() {
  if (millis() - lastGlanceMs < PATROL_GLANCE_MS) return;
  lastGlanceMs = millis();

  stopMotors();
  bool right = glanceRightNext;
  glanceRightNext = !glanceRightNext;

  float cm = lookAt(right ? SERVO_RIGHT : SERVO_LEFT);
  sonar.write(SERVO_FRONT);
  delay(SERVO_SETTLE_MS);

  Serial.print("glance "); Serial.print(right ? "R=" : "L=");
  Serial.println(cm, 0);

  if (cm < SIDE_BLIND_CM || cm < EXIT_OPEN_CM) return; // nothing worth turning for

  if (!confirmExit(right ? SERVO_RIGHT : SERVO_LEFT)) {
    Serial.println("glance did not confirm -> carrying on");
    return;
  }

  exitFound = true;
  Serial.println("EXIT spotted while driving -> heading out");
  if (right) turnRight(TURN_90_MS); else turnLeft(TURN_90_MS);
  escapeForward();
}

// Obstacle sighted: brake, back off if it is close, scan, then turn toward whichever
// side is clearer.
// irSide: -1 front-left IR tripped, +1 front-right IR tripped, 0 neither/both.
//
// The robot never backs up. It carries no rear sensor, so reversing is the one move it
// makes blind, and the pivot it would be making room for works from a standstill
// anyway: a pivot in place sweeps the chassis within its own footprint and needs
// clearance at the sides, not ahead.
void avoidObstacle(int irSide) {
  brakeHard();
  // The settle only exists so the servo sweep measures a chassis that has stopped
  // rocking. With no sweep coming, waiting here just parks the robot in front of the
  // obstacle.
  if (SCAN_BEFORE_TURN) delay(BRAKE_SETTLE_MS);
  markBlocked(DIR_FRONT);

  // Back off a fixed 10 cm before doing anything else. The pivot that follows sweeps
  // the chassis corners 3 cm past where the sonar was pointing, and this buys that
  // clearance back without the robot having to judge whether it needs it.
  reverseCm(REVERSE_CM);
  delay(BRAKE_SETTLE_MS);

  // The first obstacle of the run gets a half turn and nothing else: whatever is ahead
  // was there when the robot was set down, so the ground behind it is the one direction
  // known to be open. Every sighting after this one is scanned and decided normally.
  if (firstSighting) {
    firstSighting = false;
    Serial.println("first sighting -> turning around");
    turnAround();
    escapeForward();
    return;
  }

  // Side-stepping is not working if it keeps happening: stop guessing and go with
  // measurements instead of a rule.
  if (++consecutiveAvoids >= MAX_CONSECUTIVE_AVOIDS) {
    consecutiveAvoids = 0;
    Serial.println("boxed in -> sweeping for the farthest opening");
    escapeToFarthest(irSide);
    return;
  }

  float left = SIDE_UNKNOWN_CM, right = SIDE_UNKNOWN_CM;
  if (SCAN_BEFORE_TURN) {
    left = lookAt(SERVO_LEFT);
    right = lookAt(SERVO_RIGHT);
    sonar.write(SERVO_FRONT);
    delay(SERVO_SETTLE_MS);
  }

  // A reading from inside the robot's own footprint says nothing about the room.
  bool leftValid = left >= SIDE_BLIND_CM;
  bool rightValid = right >= SIDE_BLIND_CM;
  bool leftBlocked = leftValid && left < SIDE_MIN_CM;
  bool rightBlocked = rightValid && right < SIDE_MIN_CM;

  if (leftBlocked) markBlocked(DIR_LEFT);
  if (rightBlocked) markBlocked(DIR_RIGHT);
  // No markClear() here on purpose. A wall the robot has just backed away from often
  // scans clear from the new spot, and clearing the entry would erase the very memory
  // that keeps it from pivoting straight back into that wall. Let it decay instead.

  // Nothing within the arena can read this far, so a side that does is the gap. Take
  // it: this is the one case where the goal beats every avoidance rule, since the
  // robot is being sent at open floor rather than around an obstacle.
  if ((leftValid && left >= EXIT_OPEN_CM) || (rightValid && right >= EXIT_OPEN_CM)) {
    bool exitRight = right >= left;
    Serial.print("possible exit at "); Serial.print(exitRight ? "right " : "left ");
    Serial.print(exitRight ? right : left, 0); Serial.println("cm -> confirming");
    if (confirmExit(exitRight ? SERVO_RIGHT : SERVO_LEFT)) {
      exitFound = true;
      Serial.println("EXIT confirmed -> heading out");
      if (exitRight) turnRight(TURN_90_MS); else turnLeft(TURN_90_MS);
      escapeForward();
      return;
    }
    Serial.println("did not confirm -> scattered echo, carrying on normally");
  }

  // A side we already hit recently counts as blocked even if this scan looks clear,
  // which is what stops the robot from turning back into the wall it just left.
  bool avoidLeft = leftBlocked || recentlyBlocked(DIR_LEFT);
  bool avoidRight = rightBlocked || recentlyBlocked(DIR_RIGHT);

  Serial.print("scan L="); Serial.print(left, 0);
  if (!leftValid) Serial.print("(self)");
  Serial.print(" R="); Serial.print(right, 0);
  if (!rightValid) Serial.print("(self)");
  Serial.print(" memL="); Serial.print((int)recentlyBlocked(DIR_LEFT));
  Serial.print(" memR="); Serial.println((int)recentlyBlocked(DIR_RIGHT));

  if (avoidLeft && avoidRight) {
    turnAround();
    escapeForward();
    return;
  }

  if (avoidLeft) { turnRight(TURN_90_MS); escapeForward(); return; }
  if (avoidRight) { turnLeft(TURN_90_MS); escapeForward(); return; }

  // Both sides are open, so this is a free choice and TURN_POLICY makes it. A tripped
  // IR overrides every policy: that is a real obstacle within centimetres on that
  // side, not a preference.
  bool leftVisited = recentlyVisited(heading - 1);
  bool rightVisited = recentlyVisited(heading + 1);

  Serial.print("heading="); Serial.print(heading);
  Serial.print(" visL="); Serial.print((int)leftVisited);
  Serial.print(" visR="); Serial.println((int)rightVisited);

  bool goRight;
  bool sidesEqual = fabs(left - right) <= SIDE_ADVANTAGE_CM;
  if (irSide != 0) {
    goRight = irSide < 0;
  } else if (TURN_POLICY == POLICY_RANDOM) {
    goRight = random(2) == 1;
  } else if (TURN_POLICY == POLICY_ROOMIER) {
    goRight = sidesEqual ? true : right > left;
  } else if (TURN_POLICY == POLICY_LEFT) {
    goRight = false;
  } else if (TURN_POLICY == POLICY_RIGHT) {
    goRight = true;
  } else if (TURN_POLICY == POLICY_SMART && leftVisited != rightVisited) {
    goRight = leftVisited;
  } else if (TURN_POLICY == POLICY_SMART && !sidesEqual) {
    goRight = right > left;
  } else {
    goRight = !lastTurnRight;
  }

  if (goRight) turnRight(TURN_90_MS);
  else turnLeft(TURN_90_MS);

  escapeForward();
}

// Pings five times on the given wiring and reports how many came back. A sensor wired
// backwards answers nothing at all, which is what tells the two orders apart.
int probeSonar(int trig, int echo) {
  applySonarPins(trig, echo);
  int answered = 0;
  for (int i = 0; i < 5; i++) {
    float cm = pingCm();
    Serial.print("selftest TRIG="); Serial.print(trig == A0 ? "A0" : "A1");
    Serial.print(" ping "); Serial.print(i);
    Serial.print(": echo_us="); Serial.print(lastEchoUs);
    Serial.print(" cm="); Serial.println(cm, 1);
    if (lastEchoUs > 0) answered++;
    delay(PING_GAP_MS);
  }
  return answered;
}

void setup() {
  Serial.begin(9600);

  // Every PWM constant in this sketch is 0-255, so pin the resolution instead of
  // inheriting whatever a previously flashed sketch left in the IDE's habits.
  analogWriteResolution(8);

  pinMode(LEFT_PWM, OUTPUT); pinMode(LEFT_A, OUTPUT); pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT); pinMode(RIGHT_A, OUTPUT); pinMode(RIGHT_B, OUTPUT);
  stopMotors();

  applySonarPins(trigPin, echoPin);

  // Pull-ups first: attachEncoders() carries over the pull-up but cannot add one.
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);
  attachEncoders();
  resetStraight();

  pinMode(IR_FRONT_LEFT, INPUT_PULLUP);
  pinMode(IR_FRONT_RIGHT, INPUT_PULLUP);

  sonar.attach(SERVO_PIN);
  sonar.write(SERVO_FRONT);
  delay(300); // the horn has to actually reach centre before the first ping is taken

  // Without a seed random() replays the same sequence after every reset, which is
  // the one thing a random swerve must not do: the robot would retrace the same path
  // out of the same corner every run. A0 is the echo pin and floats between pings,
  // so its low bits plus the elapsed microseconds differ from boot to boot.
  randomSeed(analogRead(A0) ^ micros());

  // A wrong IR_ACTIVE_LOW makes every sensor read as permanently blocked, and the
  // robot then brakes on every pass of loop() and never drives anywhere. Print the raw
  // pin states with nothing in front of the robot: they should read as NOT triggered.
  Serial.print("IR raw (nothing in front expected): frontL="); Serial.print(digitalRead(IR_FRONT_LEFT));
  Serial.print(" frontR="); Serial.print(digitalRead(IR_FRONT_RIGHT));
  Serial.print(" | IR_ACTIVE_LOW="); Serial.print((int)IR_ACTIVE_LOW);
  Serial.print(" -> triggered now? frontL="); Serial.print((int)irTriggered(IR_FRONT_LEFT));
  Serial.print(" frontR="); Serial.println((int)irTriggered(IR_FRONT_RIGHT));
  if (irTriggered(IR_FRONT_LEFT) || irTriggered(IR_FRONT_RIGHT)) {
    Serial.println("WARNING: a front IR reads triggered with nothing there. The robot will");
    Serial.println("brake on every pass and never drive. Flip IR_ACTIVE_LOW or check wiring.");
  }

  Serial.println("autonomous obstacle avoidance start");
  // No settling pause here on purpose: the robot drives the moment it is powered up.
  // Put it down facing where it should go before switching it on.

  // A mute sonar reads as open floor everywhere, so the robot would drive off and hit
  // the first thing in its path at full speed. Check the sensor answers at all before
  // trusting it, and refuse to drive if it does not: standing still with an error on
  // the serial line is the more useful failure.
  int answered = probeSonar(A1, A0);
  if (answered == 0) {
    Serial.println("no echo on TRIG=A1 ECHO=A0, trying them the other way round");
    answered = probeSonar(A0, A1);
  }
  if (answered > 0) {
    Serial.print("sonar pins: TRIG="); Serial.print(trigPin == A0 ? "A0" : "A1");
    Serial.print(" ECHO="); Serial.println(echoPin == A0 ? "A0" : "A1");
  }
  if (answered == 0) {
    sonarDead = SONAR_REQUIRED;
    Serial.println("SONAR DEAD: no echo on any of 5 pings.");
    Serial.println("Both pin orders were tried. Check the 5V supply and the ground the");
    Serial.println("module shares with the board, then reset.");
    Serial.println(SONAR_REQUIRED ? "Not driving (SONAR_REQUIRED)."
                                  : "Driving anyway on the IR alone (SONAR_REQUIRED false).");
  } else {
    Serial.print("sonar ok, "); Serial.print(answered); Serial.println("/5 pings answered");
  }
}

unsigned long lastDebugMs = 0;

void loop() {
  // Set by the startup self test. Driving on a sensor that never answers is how the
  // robot ends up crashing into the first wall at full speed.
  if (sonarDead) {
    stopMotors();
    if (millis() - lastDebugMs >= 1000) {
      lastDebugMs = millis();
      float cm = pingCm();
      Serial.print("SONAR DEAD, halted. retry echo_us="); Serial.print(lastEchoUs);
      Serial.print(" cm="); Serial.println(cm, 1);
      if (lastEchoUs > 0) { sonarDead = false; Serial.println("sonar answered -> resuming"); }
    }
    return;
  }

  bool leftIr = irTriggered(IR_FRONT_LEFT);
  bool rightIr = irTriggered(IR_FRONT_RIGHT);

  // An IR hit needs no second opinion: brake before the sonar read, because pingCm()
  // blocks for up to ECHO_TIMEOUT_US and the robot would spend all of it still
  // driving at the thing the IR has already found.
  if (leftIr || rightIr) brakeHard();

  float frontCm = frontDistance();
  bool sighted = leftIr || rightIr || frontCm < FRONT_DETECT_CM;

  if (sighted) {
    // Brake on the very first reading, before any serial print or scan: stopping on
    // a false echo costs a moment, missing a real one costs a crash.
    brakeHard();

    // Now that it is standing still, re-measure. A lone bad echo would otherwise
    // leave the robot parked in front of empty floor.
    if (!leftIr && !rightIr) {
      delay(BRAKE_SETTLE_MS);
      frontCm = pingCmConfirm();
      if (frontCm >= FRONT_DETECT_CM) {
        Serial.print("false echo, resuming at "); Serial.print(frontCm, 0); Serial.println("cm");
        sighted = false;
      }
    }
  }

  if (sighted) {
    int irSide = 0;
    if (leftIr && !rightIr) irSide = -1;
    else if (rightIr && !leftIr) irSide = 1;

    avoidObstacle(irSide);
  } else {
    // Clear road: the front is known good and the maneuver streak is over.
    consecutiveAvoids = 0;
    markClear(DIR_FRONT);
    markHeadingVisited();
    driveStraight();
    patrolGlance();
  }

  if (millis() - lastDebugMs >= 500) {
    lastDebugMs = millis();
    Serial.print("front="); Serial.print(frontCm, 0);
    Serial.print("cm echo_us="); Serial.print(lastEchoUs);
    Serial.print(" irL="); Serial.print(digitalRead(IR_FRONT_LEFT));
    Serial.print(" irR="); Serial.print(digitalRead(IR_FRONT_RIGHT));
    Serial.print(" blocked="); Serial.print((int)(leftIr || rightIr));
    Serial.print(" avoids="); Serial.print(consecutiveAvoids);
    Serial.print(" pwm="); Serial.print(straightPwm);
    Serial.print(" encFault="); Serial.print((int)encoderFault);
    Serial.print(" exit="); Serial.print((int)exitFound);
    Serial.print(" state="); Serial.println(sonarDead ? "HALT_SONAR_DEAD"
                                            : sighted ? "AVOIDING" : "DRIVING");
  }
}
