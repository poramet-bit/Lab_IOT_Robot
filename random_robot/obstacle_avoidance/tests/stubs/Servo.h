#pragma once
struct Servo {
  int angle=90;
  void attach(int){}
  void write(int value){angle=value;}
};
