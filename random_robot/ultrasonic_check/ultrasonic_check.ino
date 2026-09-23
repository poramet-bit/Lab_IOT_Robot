#define TRIG_PIN A0
#define ECHO_PIN A1

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("ultrasonic check start");
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    Serial.println("no echo (check wiring / timeout)");
  } else {
    float distanceCm = duration * 0.0343 / 2;
    Serial.print("distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
  }

  delay(300);
}
