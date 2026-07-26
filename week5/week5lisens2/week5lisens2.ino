void setup() {
  // put your setup code here, to run once:
  //variable resistor
  //1 -> 3.3v/5v
  //2 -> A0
  //3 -> GD
  

  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  int val_a = 0;
  val_a = analogRead(A0);

  int val_b  = 0;
  val = map(val_a,0,4095,0,100);

  Serial.println(val_b);
  delay(500);
}
