#include <Wire.h>
#include <ArduinoBLE.h>

// Declare the custom type and prototype before Arduino generates function prototypes.
enum Motion { IDLE, LEFT, RIGHT };
void startMotion(Motion requested);

// BLE spin test: L/R spins in place, S/STOP brakes and sends one summary.
// Wiring confirmed by user: code channel L = physical right, R = physical left.
// Keep the robot still during gyro calibration. No LED animation.
const int ENA = 5;
const int IN1 = 7;
const int IN2 = 6;
const int ENB = 10;
const int IN3 = 8;
const int IN4 = 9;
const int ENCODER_L = 3;
const int ENCODER_R = 11;
const int TURN_SPEED = 40000; // Copied from robotcurclerun_raw.
const int DISK_SLOTS = 20;
const float CM_PER_PULSE = 3.14159265f * 6.5f / DISK_SLOTS;
const float TRACK_WIDTH_CM = 14.5f;
const unsigned long BRAKE_MS = 150;
volatile unsigned long pulse_count_L = 0;
volatile unsigned long pulse_count_R = 0;
void isr_count_L() { pulse_count_L++; }
void isr_count_R() { pulse_count_R++; }

BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 200);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
bool ble_ready = false;
bool running = false;
bool braking = false;
unsigned long brake_started = 0;
Motion motion = IDLE;
unsigned long motion_started = 0;
unsigned long drive_ms = 0;
int pwm_left = 0;
int pwm_right = 0;
bool summary_pending = false;
bool gyro_valid = false;
bool gyro_saturated = false;
String report;
unsigned int report_offset = 0;
unsigned long last_report_chunk = 0;

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void brakeMotors() {
  if (braking) return;
  if (running) {
    drive_ms = millis() - motion_started;
    summary_pending = true;
  }
  running = false;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 65535);
  analogWrite(ENB, 65535);
  braking = true;
  brake_started = millis();
}

void sendTelemetry(const String &message) {
  Serial.println(message);
  if (ble_ready && txChar.subscribed()) {
    txChar.writeValue(message.c_str(), message.length());
  }
}

const bool USE_MPU6050 = true; // ต่อสาย A4(SDA)/A5(SCL) แล้ว

const int MPU_ADDR = 0x68;
bool has_mpu = false;
float gyro_z_offset = 0;
float current_yaw = 0;
unsigned long last_gyro_time = 0;

bool initMPU6050() {
  if (!USE_MPU6050) {
    Serial.println("[IMU] MPU-6050 disabled (USE_MPU6050 = false).");
    has_mpu = false;
    return false;
  }

  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_UNOR4_MINIMA)
  Wire.setWireTimeout(10000, true);
  #endif

  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("[IMU] MPU-6050 not detected.");
    has_mpu = false;
    return false;
  }

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Set Gyro full scale range to +/- 250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Serial.println("[IMU] MPU-6050 detected! Calibrating gyro Z (keep robot still)...");
  long sum_z = 0;
  int samples = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    if (Wire.available() >= 2) {
      int16_t raw_z = (Wire.read() << 8) | Wire.read();
      sum_z += raw_z;
      samples++;
    }
    delay(3);
  }
  if (samples == 0) { has_mpu = false; return false; }
  gyro_z_offset = (float)sum_z / samples;
  current_yaw = 0;
  last_gyro_time = micros();
  has_mpu = true;
  Serial.println("[IMU] Calibration complete! Gyro reading (log-only, no correction) active.");
  return true;
}

void updateYaw() {
  if (!has_mpu) return;

  unsigned long now = micros();
  float dt = (now - last_gyro_time) / 1000000.0;
  last_gyro_time = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  if (Wire.available() >= 2) {
    int16_t raw_z = (Wire.read() << 8) | Wire.read();
    if (raw_z >= 32700 || raw_z <= -32700) gyro_saturated = true;
    float rate_z = (raw_z - gyro_z_offset) / 131.0; // 131 LSB/(deg/s)
    if (abs(rate_z) > 0.15) { // Filter sensor noise
      current_yaw += rate_z * dt;
    }
  } else {
    gyro_valid = false;
  }
}

void resetYaw() {
  current_yaw = 0;
  last_gyro_time = micros();
}


// Refuse mode changes until S finishes the current measurement and report.
void startMotion(Motion requested) {
  if (requested == IDLE || running || braking || report.length() > 0) return;
  noInterrupts();
  pulse_count_L = 0;
  pulse_count_R = 0;
  interrupts();
  resetYaw();
  gyro_valid = has_mpu;
  gyro_saturated = false;
  motion = requested;
  pwm_left = TURN_SPEED;
  pwm_right = TURN_SPEED;
  // Spin right: physical right backward, physical left forward; reverse for left.
  const bool codeLeftForward = requested == LEFT;
  digitalWrite(IN1, codeLeftForward ? LOW : HIGH);
  digitalWrite(IN2, codeLeftForward ? HIGH : LOW);
  digitalWrite(IN3, codeLeftForward ? HIGH : LOW);
  digitalWrite(IN4, codeLeftForward ? LOW : HIGH);
  analogWrite(ENA, pwm_left);
  analogWrite(ENB, pwm_right);
  motion_started = millis();
  running = true;
}

void buildSummary() {
  noInterrupts();
  unsigned long left = pulse_count_L;
  unsigned long right = pulse_count_R;
  interrupts();
  float cm_left = left * CM_PER_PULSE;
  float cm_right = right * CM_PER_PULSE;
  float seconds = drive_ms / 1000.0f;
  bool valid_yaw = gyro_valid && !gyro_saturated;
  report = "\n=== SUMMARY ===\nMODE=" + String(motion == LEFT ? "LEFT" : "RIGHT");
  report += "\nDRIVE_MS=" + String(drive_ms) + " BRAKE_MS=" + String(BRAKE_MS);
  report += "\nPWM_L=" + String(pwm_left) + " PWM_R=" + String(pwm_right);
  report += "\nENC_L=" + String(left) + " ENC_R=" + String(right);
  report += "\nENC_DIFF_L-R=" + String((long)left - (long)right);
  report += "\nREV_L=" + String((float)left / DISK_SLOTS, 3) + " REV_R=" + String((float)right / DISK_SLOTS, 3);
  report += "\nCM_L=" + String(cm_left, 2) + " CM_R=" + String(cm_right, 2);
  report += "\nCM_DIFF_L-R=" + String(cm_left - cm_right, 2);
  report += "\nYAW_DEG=" + (valid_yaw ? String(current_yaw, 2) : String("NA"));
  report += " GYRO=" + String(!has_mpu ? "MISSING" : (!gyro_valid ? "READ_ERROR" : (gyro_saturated ? "SATURATED" : "OK")));
  report += "\nTURN_MS=" + String(drive_ms);
  report += "\nENC_TURN_ABS_DEG_EST=" + String((cm_right + cm_left) / TRACK_WIDTH_CM * 180.0f / 3.14159265f, 2);
  if (valid_yaw && abs(current_yaw) >= 5.0f && seconds > 0) {
    float deg = abs(current_yaw);
    report += "\nAVG_DEG_PER_SEC=" + String(deg / seconds, 2);
    report += "\nEST_MS_90=" + String(seconds * 1000.0f * 90.0f / deg, 0);
    report += " EST_MS_180=" + String(seconds * 1000.0f * 180.0f / deg, 0);
    report += "\nEST_MS_270=" + String(seconds * 1000.0f * 270.0f / deg, 0);
    report += " EST_MS_360=" + String(seconds * 1000.0f * 360.0f / deg, 0);
    report += "\nTime estimates include stopping angle; retest for acceleration/braking.";
  } else {
    report += "\nTurn time estimate unavailable: invalid gyro or angle <5 deg.";
  }
  report += "\nWheel distances are encoder estimates, not lateral drift.\n=== END ===\n";
  Serial.print(report);
  report_offset = 0;
  summary_pending = false;
  motion = IDLE;
}

// One summary per stop, split into paced 20-byte UART notifications for BLE MTU.
void flushReport() {
  if (!ble_ready || report.length() == 0 || !txChar.subscribed()) return;
  if (millis() - last_report_chunk < 30) return;
  unsigned int count = report.length() - report_offset;
  if (count > 20) count = 20;
  if (txChar.writeValue((const uint8_t *)report.c_str() + report_offset, count)) {
    report_offset += count;
    last_report_chunk = millis();
    if (report_offset >= report.length()) { report = ""; report_offset = 0; }
  }
}

void setup() {
  Serial.begin(9600);
  analogWriteResolution(16);
  const int motorPins[] = {ENA, ENB, IN1, IN2, IN3, IN4};
  for (int pin : motorPins) pinMode(pin, OUTPUT);
  stopMotors();
  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), isr_count_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), isr_count_R, RISING);
  initMPU6050(); // Keep robot still during calibration.
  ble_ready = BLE.begin();
  if (!ble_ready) {
    Serial.println("BLE initialization failed. Motors remain stopped.");
    return;
  }
  BLE.setLocalName("IOT-SPIN");
  BLE.setDeviceName("IOT-SPIN");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(txChar);
  uartService.addCharacteristic(rxChar);
  BLE.addService(uartService);
  BLE.advertise();
  Serial.println("Ready: connect BLE UART, subscribe to TX, send L / R to spin, then S to brake and report.");
}

void loop() {
  if (ble_ready) {
    BLE.poll();
    BLEDevice central = BLE.central();
    if (central && central.connected()) {
      if (rxChar.written()) {
        String command = "";
        for (int i = 0; i < rxChar.valueLength(); i++) command += (char)rxChar.value()[i];
        command.trim();
        command.toUpperCase();
        if (command == "STOP" || command == "S") {
          brakeMotors();
        } else if (command == "L") {
          startMotion(LEFT);
        } else if (command == "R") {
          startMotion(RIGHT);
        }
      }
    } else if (running) {
      brakeMotors();
    }
  }
  updateYaw();
  if (braking && millis() - brake_started >= BRAKE_MS) {
    stopMotors();
    braking = false;
    if (summary_pending) buildSummary();
  }
  flushReport();
  delay(2);
}
