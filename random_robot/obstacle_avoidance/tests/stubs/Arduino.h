#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
constexpr int A0=14,A1=15,A2=16,A3=17,A4=18,A5=19;
constexpr int LOW=0,HIGH=1,INPUT=0,OUTPUT=1,INPUT_PULLUP=2;
static uint32_t fakeTime=0;
static int pwm[32]={}, pins[32]={}, modes[32]={}, pwmBits=0;
static unsigned long fakeEchoUs=11662;
static void (*pingHook)()=nullptr;
inline uint32_t millis(){return fakeTime;}
inline void delayMicroseconds(unsigned int){}
inline void pinMode(int pin,int mode){modes[pin]=mode;if(mode==INPUT_PULLUP)pins[pin]=HIGH;}
inline void analogWriteResolution(int n){pwmBits=n;}
inline void analogWrite(int pin,int n){assert(n>=0 && n<=255);pwm[pin]=n;}
inline void digitalWrite(int pin,int n){
  if(pin>=6 && pin<=9 && pins[pin]!=n) assert(pwm[pin<=7?5:10]==0);
  pins[pin]=n;
}
inline int digitalRead(int pin){return pins[pin];}
inline int constrain(int v,int lo,int hi){return v<lo?lo:(v>hi?hi:v);}
inline unsigned long pulseIn(int pin,int,unsigned long timeout){
  assert(pin==A0); fakeTime+=fakeEchoUs?fakeEchoUs/1000:timeout/1000;
  if(pingHook){auto hook=pingHook;pingHook=nullptr;hook();}
  return fakeEchoUs;
}
struct FakeSerial {
  std::string input,output;
  int space=16;
  void begin(int){}
  operator bool()const{return true;}
  int available()const{return int(input.size());}
  int read(){int c=input.front();input.erase(0,1);return c;}
  int availableForWrite()const{return space;}
  size_t write(const uint8_t* data,size_t n){assert(n<=size_t(space));output.append((const char*)data,n);return n;}
};
static FakeSerial Serial;
