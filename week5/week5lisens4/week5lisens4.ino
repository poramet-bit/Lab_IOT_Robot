//serial plotter ep6 supep 6

void setup() {
  // put your setup code here, to run once:
  //variable resistor
  //1 -> 3.3v/5v
  //2 -> A0
  //3 -> GD
  
  /*
  10 bit = 0-1023 
  11 bit = 0-2047
  12 bit = 0-4095
  13 bit  = 0-8191
  14 bit = 0-16383
  15 bit = 0-32767
  16 bit = 0-65535

  */
  
  Serial.begin(9600);
  analogReadResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:

  int val_a = 0;
  val_a = analogRead(A0);

 /*
  formula
 (max voltage/resoultion of ADC)/variable a  
  */

  
  double val_b = 0;
  val_b = (3.3/4096)*a;
    
  /*
  int val_b  = 0;
  val = map(val_a,0,4095,0,100);
  */
  Serial.println(val_b);
  delay(500);
}
