#include "Arduino_LED_Matrix.h"
#include "/home/poramet/Documents/Lab_IOT_Robot/animation/animation.h"

ArduinoLEDMatrix matrix;

/*
  analog(0)
  analog(1024) 1.25 V
  analog(2047) 2.5 V
  analog(3071) 3.75 V
  analog(4095) 5 V
  */

//Left motor
int ENA = 10;
int IN1 = 9;
int IN2 = 8;
//Right motor
int ENB = 5;
int IN3 = 7;
int IN4 = 6;

// LM393 Speed Sensor (Encoder) - Pin 11: Left wheel (ENA), Pin 3: Right wheel (ENB)
const int ENCODER_L = 11;
const int ENCODER_R = 3;
const int DISK_SLOTS = 20; // 20 slots per revolution

// Path-following geometry.
const float WHEELBASE_M = 0.10; // 10 cm between wheel centers (still theoretical, not field-calibrated)
// PULSES_PER_METER field-calibrated: robot covered measured 450 cm in ~473 avg
// pulses. Theoretical 65mm-wheel estimate was 97.9 pulses/m, off by ~7% (real
// wheel slip / effective rolling diameter differs from nominal).
const float PULSES_PER_METER = 105.1;
// Point turns scrub the tires sideways instead of rolling cleanly, so pure
// wheelbase geometry under-predicts the pulses needed per degree. Empirical
// tuning trail: uncorrected -> commanded 90 deg gave ~45 deg real (needed ~2x);
// factor 2.0 -> commanded 90 deg gave ~105 deg real (overshoot by 105/90);
// corrected to 2.0 * (90/105) ~= 1.71.
const float SPIN_SLIP_FACTOR = 1.71;
const float PULSES_PER_DEGREE = (WHEELBASE_M * PI / 360.0) * PULSES_PER_METER * SPIN_SLIP_FACTOR; // ~0.183

// Fixed route: each step drives straight for distance_m, does a 360-degree
// flourish spin, then (if has_turn) an extra 90-degree turn before the next
// step. The last step has no turn — the robot stops after its spin.
struct PathStep {
  float distance_m;
  bool spin_clockwise;
  bool has_turn;
  bool turn_clockwise;
};

const PathStep PATH[] = {
  { 4.5, true,  true,  true },  // straight 4.5 m, spin right 360, turn right 90
  { 5.7, false, true,  true },  // straight 5.7 m, spin left 360, turn right 90
  { 4.5, true,  true,  true },  // straight 4.5 m, spin right 360, turn right 90
  { 5.7, false, true,  true },  // straight 5.7 m, spin left 360, turn right 90
  { 4.5, true,  false, false }, // straight 4.5 m, spin right 360, then stop
};
const int PATH_LEN = sizeof(PATH) / sizeof(PATH[0]);
int path_index = 0;
bool path_done = false;

volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

unsigned long milestone_start_L = 0;
unsigned long milestone_start_R = 0;

// Interrupt Service Routines (ISRs)
void isr_count_L() {
  pulse_count_L++;
}

void isr_count_R() {
  pulse_count_R++;
}

// Functions to get revolutions and pulses
float getRevolutionsL() {
  return (float)pulse_count_L / DISK_SLOTS;
}

float getRevolutionsR() {
  return (float)pulse_count_R / DISK_SLOTS;
}

unsigned long getPulsesL() {
  return pulse_count_L;
}

unsigned long getPulsesR() {
  return pulse_count_R;
}

void resetEncoders() {
  pulse_count_L = 0;
  pulse_count_R = 0;
}

// Auto-Balance Proportional Gain (Kp)
float Kp = 1.4;

void setBalanceKp(float k) {
  Kp = k;
}

// Hardware Trim Ratio for Left Motor (calibrated: Left motor ENA is naturally faster)
float MOTOR_L_RATIO = 0.825;

void setMotorLRatio(float ratio) {
  MOTOR_L_RATIO = ratio;
}

void printEncoderStatus() {
  Serial.print("Left Pulses: ");
  Serial.print(pulse_count_L);
  Serial.print(" (Rev: ");
  Serial.print(getRevolutionsL(), 2);
  Serial.print(") | Right Pulses: ");
  Serial.print(pulse_count_R);
  Serial.print(" (Rev: ");
  Serial.print(getRevolutionsR(), 2);
  Serial.println(")");
}

int speed_motorL = 0;
int speed_motorR = 0;

int B_L = 0;
int B_R = 0;

void setup() {
  //analogWriteResolution(12);
  // put your setup code here, to run once:
  Serial.begin(9600);
  matrix.begin();
  //Left motor set up
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  //Right motor set up
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  //LM393 Encoder set up
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);
}

void increse_speed() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  for (int i = 2050; i < 4096; i++) {
    analogWrite(ENA, i - B_L);
    analogWrite(ENB, i - B_R);
  }
  for (int i = 4095; i > 2051; i--) {
    analogWrite(ENA, i - B_L);
    analogWrite(ENB, i - B_R);
  }
}

void forward(int speed_motorL = 200, int speed_motorR = 200, int B_L = 0, int B_R = 0) {
  // Motor direction
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  unsigned long start_L = pulse_count_L;
  unsigned long start_R = pulse_count_R;

  // Apply hardware base trim ratio to Left motor (which runs naturally faster)
  int base_L = (int)((speed_motorL - B_R) * MOTOR_L_RATIO);
  int base_R = speed_motorR - B_L;

  int total_frames = sizeof(walk) / sizeof(walk[0]);
  for (int i = 0; i < total_frames; i++) {
    // Auto-Balance: compute pulse difference and adjust PWM in real-time
    long delta_L = pulse_count_L - start_L;
    long delta_R = pulse_count_R - start_R;
    long error = delta_L - delta_R; // >0: Left is faster, <0: Right is faster

    int adjustment = 0;
    // Deadband: within 1 pulse, consider straight (prevents snaking oscillation)
    if (abs(error) > 1) {
      adjustment = (int)(Kp * error);
      // Clamp adjustment to prevent stalling or extreme changes
      adjustment = constrain(adjustment, -35, 35);
    }

    int current_L = constrain(base_L - adjustment, 90, 255);
    int current_R = constrain(base_R + adjustment, 90, 255);

    analogWrite(ENA, current_L);
    analogWrite(ENB, current_R);

    loadFlippedYFrame(walk[i]);
    delay(walk[i][3]); // 66 ms frame delay
  }
}

void backward(int speed_motorL = 200, int speed_motorR = 200, int B_L = 0, int B_R = 0) {
  // Motor direction
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

  unsigned long start_L = pulse_count_L;
  unsigned long start_R = pulse_count_R;

  // Apply hardware base trim ratio to Left motor
  int base_L = (int)((speed_motorL - B_R) * MOTOR_L_RATIO);
  int base_R = speed_motorR - B_L;

  int total_frames = sizeof(walk) / sizeof(walk[0]);
  for (int i = 0; i < total_frames; i++) {
    // Auto-Balance: compute pulse difference and adjust PWM in real-time
    long delta_L = pulse_count_L - start_L;
    long delta_R = pulse_count_R - start_R;
    long error = delta_L - delta_R;

    int adjustment = 0;
    if (abs(error) > 1) {
      adjustment = (int)(Kp * error);
      adjustment = constrain(adjustment, -35, 35);
    }

    int current_L = constrain(base_L - adjustment, 90, 255);
    int current_R = constrain(base_R + adjustment, 90, 255);

    analogWrite(ENA, current_L);
    analogWrite(ENB, current_R);

    loadFlippedXFrame(walk[i]);
    delay(walk[i][3]); // 66 ms frame delay
  }
}



void right_turn(int speed_motorR = 200,int speed_motorL = 0, int B_L = 0, int B_R = 0) {


  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  //Define speed for motor
 //left motor
  analogWrite(ENA, speed_motorL - B_R);
  //right motor
  analogWrite(ENB, speed_motorR - B_L);
  Serial.print("Right turn");
  int total_frames = sizeof(side_walk) / sizeof(side_walk[0]);
  for (int i = 0; i < total_frames; i++) {
    loadFlippedYFrame(side_walk[i]);
    delay(side_walk[i][3]); // 66 ms frame delay
  }
}





void left_turn(int speed_motorL = 0, int speed_motorR = 200, int B_L = 0, int B_R = 0) {

  // Define parameter for L298N
  // Right wheel drives forward while left wheel stays stationary, pivoting
  // the chassis toward the (stationary) left side. Driving the left wheel
  // instead (old code) pivots toward the right side, i.e. turns the wrong way.
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  //Define speed for motor

  //left motor
  analogWrite(ENA, speed_motorL - B_R);
  //right motor
  analogWrite(ENB, speed_motorR - B_L);
  Serial.println("Left turn");

 
  // Redo forward order with swapped X and Y axes
  int total_frames = sizeof(side_walk) / sizeof(side_walk[0]);
  for (int i = 0; i < total_frames; i++) {
    loadFlippedXYFrame(side_walk[i]);
    delay(side_walk[i][3]); // 66 ms frame delay
  }
}

void left_turnback(int speed_motorL = 200, int speed_motorR = 200, int B_L = 0, int B_R = 0) {

  // Define parameter for L298N
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

  //Define speed for motor
  //left motor
  analogWrite(ENA, speed_motorL - B_R);
  //right motor
  analogWrite(ENB, speed_motorR - B_L);
  int total_frames = sizeof(radar) / sizeof(radar[0]);
  for (int i = 0; i < total_frames; i++) {
    loadFlippedYFrame(radar[i]);
    delay(radar[i][3]); // 66 ms frame delay
  }
}
void right_turnback(int speed_motorL = 200, int speed_motorR = 200, int B_L = 0, int B_R = 0) {
  // Define parameter for L298N
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  //Define speed for motor
  //left motor
  analogWrite(ENA, speed_motorL - B_R);
  //right motor
  analogWrite(ENB, speed_motorR - B_L);
  int total_frames = sizeof(radar) / sizeof(radar[0]);
  for (int i = 0; i < total_frames; i++) {
    loadFlippedXYFrame(radar[i]);
    delay(radar[i][3]); // 66 ms frame delay
  }
}
// Encoder-gated in-place spin. clockwise=true turns the nose right.
// NOTE: branches intentionally inverted vs. the geometric derivation — on the
// physical robot that math had the direction backwards (confirmed on hardware).
void spinInPlace(int degrees, bool clockwise, int spinSpeed = 200) {
  if (clockwise) {
    digitalWrite(IN1, 1); digitalWrite(IN2, 0); // left backward
    digitalWrite(IN3, 0); digitalWrite(IN4, 1); // right forward
  } else {
    digitalWrite(IN1, 0); digitalWrite(IN2, 1); // left forward
    digitalWrite(IN3, 1); digitalWrite(IN4, 0); // right backward
  }

  unsigned long start_L = pulse_count_L;
  unsigned long start_R = pulse_count_R;
  long targetPulses = (long)(PULSES_PER_DEGREE * degrees + 0.5);
  unsigned long spinStart = millis();

  // Same left-motor trim as forward()/backward() — ENA runs naturally faster,
  // and spinInPlace previously drove both sides at equal PWM (see TUNING_LOG.md
  // on left-wheel-too-strong drive symptoms after adding this function).
  analogWrite(ENA, (int)(spinSpeed * MOTOR_L_RATIO));
  analogWrite(ENB, spinSpeed);

  while ((long)(pulse_count_L - start_L) + (long)(pulse_count_R - start_R) < targetPulses * 2) {
    if (millis() - spinStart > 5000) { // safety: encoder stall (see TUNING_LOG.md)
      Serial.println("spinInPlace: timeout, aborting");
      break;
    }
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Runs the spin (+ optional turn) maneuver for the current PATH step, then
// advances to the next step or marks the route complete on the last one.
void runPathStepManeuver() {
  const PathStep &step = PATH[path_index];
  Serial.print("Path step ");
  Serial.print(path_index);
  Serial.println(" maneuver");

  spinInPlace(360, step.spin_clockwise);
  delay(200);
  if (step.has_turn) {
    spinInPlace(90, step.turn_clockwise);
    delay(200);
  }

  milestone_start_L = pulse_count_L;
  milestone_start_R = pulse_count_R;
  path_index++;
  if (path_index >= PATH_LEN) {
    path_done = true;
    stop();
  }
}

void stop() {
  analogWrite(ENA, 0);

  analogWrite(ENB, 0);
  int total_frames = sizeof(heart) / sizeof(heart[0]);
  for (int i = 0; i < total_frames; i++) {
    loadFlippedYFrame(heart[i]);
    delay(heart[i][3]); // 66 ms frame delay
  }
}

void loadTransformedFrame(const uint32_t frame[4], bool flipX, bool flipY) {
  uint32_t transformed[4] = {0, 0, 0, frame[3]};

  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 12; x++) {
      int srcBit = y * 12 + x;
      int srcWord = srcBit / 32;
      int srcPos = 31 - (srcBit % 32);
      bool isSet = (frame[srcWord] >> srcPos) & 1;

      if (isSet) {
        int new_x = flipX ? (11 - x) : x;
        int new_y = flipY ? (7 - y) : y;

        int dstBit = new_y * 12 + new_x;
        int dstWord = dstBit / 32;
        int dstPos = 31 - (dstBit % 32);
        transformed[dstWord] |= (1UL << dstPos);
      }
    }
  }
  matrix.loadFrame(transformed);
}

void loadFlippedXFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, true, false);
}

void loadFlippedYFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, false, true);
}

void loadFlippedXYFrame(const uint32_t frame[4]) {
  loadTransformedFrame(frame, true, true);
}
void loop() {
  if (path_done) {
    return; // route finished, stop() already called
  }

  forward();
  delay(1000);
  printEncoderStatus();

  unsigned long traveled = ((pulse_count_L - milestone_start_L) + (pulse_count_R - milestone_start_R)) / 2;
  unsigned long target_pulses = (unsigned long)(PATH[path_index].distance_m * PULSES_PER_METER + 0.5);
  if (traveled >= target_pulses) {
    runPathStepManeuver();
  }
}
