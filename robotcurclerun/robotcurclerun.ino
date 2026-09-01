#include "Arduino_LED_Matrix.h"
#include "../animation/animation.h"

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

volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;

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





void left_turn(int speed_motorL = 200, int speed_motorR = 0,int B_L = 0, int B_R = 0) {

  // Define parameter for L298N

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);

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
  // put your main code here, to run repeatedly:
  int d_time = 1000;
  /*
  Sv(180);
  Sv(90);
  Sv(45);
  Sv(0);
  Sv(90);
  */
  
  forward();
  delay(1000);
  printEncoderStatus();
  /*
  backward();
  delay(1000);
  right_turn();
  delay(1000);
  left_turn();
  delay(1000);
  left_turnback();
  delay(1000);
  right_turnback();
  delay(1000);
  stop();
  delay(1000);
  */
  
}
