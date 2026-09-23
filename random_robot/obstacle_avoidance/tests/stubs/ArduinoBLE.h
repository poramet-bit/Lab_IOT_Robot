#pragma once
#include <string>
#include <cstring>
constexpr int BLEWrite=1,BLEWriteWithoutResponse=2,BLENotify=4;
struct BLECharacteristic {
  std::string incoming,output;
  BLECharacteristic(const char*,int,int){}
  bool written()const{return !incoming.empty();}
  int readValue(uint8_t* data,size_t n){if(n>incoming.size())n=incoming.size();memcpy(data,incoming.data(),n);incoming.erase(0,n);return int(n);}
  bool subscribed()const{return true;}
  bool writeValue(const uint8_t* data,size_t n){output.append((const char*)data,n);return true;}
};
struct BLEService {
  BLEService(const char*){}
  void addCharacteristic(BLECharacteristic&){}
};
struct FakeBLE {
  bool linked=false;
  bool begin(){return true;}
  void poll(){}
  bool connected()const{return linked;}
  void setLocalName(const char*){}
  void setDeviceName(const char*){}
  void setAdvertisedService(BLEService&){}
  void addService(BLEService&){}
  void advertise(){}
};
static FakeBLE BLE;
