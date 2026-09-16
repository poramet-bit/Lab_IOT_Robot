#include <ArduinoBLE.h>
#include "line_controller.h"

// UNO R4 WiFi, TCRT5000 AO x5, black line on yellow-brown floor.
// Physical sides are viewed from the rear toward the front.
// A4/A5 belong to the line array: do not connect the old I2C gyro here.
const int SENSOR_PINS[5] = {A5, A4, A3, A2, A1};
const int LEFT_PWM = 10, LEFT_IN1 = 8, LEFT_IN2 = 9;
const int RIGHT_PWM = 5, RIGHT_IN1 = 7, RIGHT_IN2 = 6;

// Initial values only: use v on black and floor, then tune these per sensor.
// ADC is explicitly 10-bit (0..1023). No automatic calibration.
const bool BLACK_IS_HIGH = true;
const int THRESHOLD[5] = {512, 512, 512, 512, 512};
const int HYSTERESIS = 20;
const unsigned long CONTROL_MS = 10;

LineController controller; // Speed/gain/search settings are in line_controller.h.
int raw[5] = {};
bool onBlack[5] = {};
uint8_t sensorMask = 0;
unsigned long lastControl = 0;
bool bleReady = false;
bool wasConnected = false;

BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic txChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 200);
BLECharacteristic rxChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite | BLEWriteWithoutResponse, 64);
String pendingTx;
unsigned int txOffset = 0;
unsigned long lastTx = 0;

void queueMessage(const String &message) {
  Serial.println(message);
  // Keep a bounded queue; driving never waits for a phone or notification.
  if (pendingTx.length() == 0 || txOffset == 0) {
    pendingTx = message + "\n";
    txOffset = 0;
  }
}

void flushTx() {
  if (!txChar.subscribed() || pendingTx.length() == 0 || millis() - lastTx < 30) return;
  unsigned int count = pendingTx.length() - txOffset;
  if (count > 20) count = 20;
  if (txChar.writeValue((const uint8_t *)pendingTx.c_str() + txOffset, count)) {
    txOffset += count;
    lastTx = millis();
    if (txOffset == pendingTx.length()) { pendingTx = ""; txOffset = 0; }
  }
}

void setWheel(int enable, int in1, int in2, int pwm) {
  if (pwm <= 0) {
    // Dynamic brake: hold both direction inputs equal with enable high.
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    analogWrite(enable, 65535);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enable, pwm);
  }
}

void applyOutput(const LineOutput &out) {
  setWheel(LEFT_PWM, LEFT_IN1, LEFT_IN2, out.left);
  setWheel(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, out.right);
}

void stopFollowing(const String &reason) {
  controller.stop();
  applyOutput(LineOutput{});
  queueMessage(reason);
}

void readSensors() {
  sensorMask = 0;
  for (int i = 0; i < 5; ++i) {
    raw[i] = analogRead(SENSOR_PINS[i]);
    const int evidence = BLACK_IS_HIGH ? raw[i] - THRESHOLD[i] : THRESHOLD[i] - raw[i];
    if (evidence > HYSTERESIS) onBlack[i] = true;
    else if (evidence < -HYSTERESIS) onBlack[i] = false;
    if (onBlack[i]) sensorMask |= (1 << i);
  }
}

void showSensors() {
  readSensors();
  String message = "LEFT -> RIGHT pins=A5,A4,A3,A2,A1\nRAW=";
  for (int i = 0; i < 5; ++i) {
    if (i) message += ",";
    message += String(raw[i]);
  }
  message += "\nBLACK=";
  for (int i = 0; i < 5; ++i) message += onBlack[i] ? "1" : "0";
  message += "\nPOLARITY=";
  message += BLACK_IS_HIGH ? "HIGH" : "LOW";
  queueMessage(message);
}

void handleCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command == "S" || command == "STOP") {
    stopFollowing("STOP: braked");
  } else if (command == "F" || command == "START") {
    // Repeated start must not reset a running search timeout.
    if (controller.state == LineController::STOPPED) {
      controller.start();
      queueMessage("FOLLOW: started");
    }
  } else if (command == "V") {
    if (controller.state == LineController::STOPPED) showSensors();
    else queueMessage("Send s before v");
  } else {
    queueMessage("Commands: f=start, s=stop, v=sensors while stopped");
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteResolution(16);
  analogReadResolution(10);
  const int motorPins[] = {LEFT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2};
  for (int pin : motorPins) pinMode(pin, OUTPUT);
  applyOutput(LineOutput{});
  for (int pin : SENSOR_PINS) pinMode(pin, INPUT);
  bleReady = BLE.begin();
  if (!bleReady) { Serial.println("BLE failed; motors remain braked."); return; }
  BLE.setLocalName("IOT-LINE");
  BLE.setDeviceName("IOT-LINE");
  BLE.setAdvertisedService(uartService);
  uartService.addCharacteristic(txChar);
  uartService.addCharacteristic(rxChar);
  BLE.addService(uartService);
  BLE.advertise();
  Serial.println("IOT-LINE ready. Subscribe to TX. f / s / v. Check thresholds before driving.");
}

void loop() {
  if (!bleReady) return;
  BLE.poll();
  BLEDevice central = BLE.central();
  const bool connected = central && central.connected();
  if (!connected) {
    if (wasConnected || controller.state != LineController::STOPPED) {
      stopFollowing("BLE disconnected: braked");
      pendingTx = "";
      txOffset = 0;
    }
    wasConnected = false;
    return;
  }
  wasConnected = true;
  if (rxChar.written()) {
    String command;
    for (int i = 0; i < rxChar.valueLength(); ++i) command += (char)rxChar.value()[i];
    handleCommand(command);
  }
  const unsigned long now = millis();
  if (now - lastControl >= CONTROL_MS) {
    lastControl = now;
    readSensors();
    const LineController::State previous = controller.state;
    applyOutput(controller.update(sensorMask, now));
    if (controller.state != previous) {
      if (controller.state == LineController::SEARCHING) queueMessage("SEARCH: last seen side, max 2 seconds");
      else if (controller.state == LineController::STOPPED) queueMessage("LOST: braked, send f to retry");
      else queueMessage("FOLLOW: line reacquired");
    }
  }
  flushTx();
}
