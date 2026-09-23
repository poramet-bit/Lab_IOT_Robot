#include <iostream>
#include "../obstacle_avoidance.ino"
void braked(){
  assert(pwm[5]==255 && pwm[10]==255);
  assert(pins[6]==HIGH && pins[7]==HIGH && pins[8]==HIGH && pins[9]==HIGH);
  assert(pwmLeft==0 && pwmRight==0);
}
void tick(int times=1){for(int i=0;i<times;++i){fakeTime+=10;loop();}}
void send(const char* s){rx.incoming=s;tick();}
int main(){
  modes[3]=modes[11]=-1;
  setup();assert(pwmBits==8);braked();
  assert(modes[3]==-1 && modes[11]==-1); // Former feedback pins are not configured.
  assert(modes[2]==INPUT_PULLUP); // The fourth IR must be read, not left floating.
  struct SensorCase { int pin; const char* report; };
  for(const SensorCase& sensor : {
      SensorCase{A3,"IR_FL=1 IR_FR=0 IR_BL=0 IR_BR=0"},
      SensorCase{A4,"IR_FL=0 IR_FR=1 IR_BL=0 IR_BR=0"},
      SensorCase{A5,"IR_FL=0 IR_FR=0 IR_BL=1 IR_BR=0"},
      SensorCase{2, "IR_FL=0 IR_FR=0 IR_BL=0 IR_BR=1"}}) {
    pins[sensor.pin]=LOW; queueStatus(); assert(strstr(report,sensor.report)!=nullptr);
    pins[sensor.pin]=HIGH;
  }
  for(int sensor : {A3,A4,A5,2}) for(const char* turn : {"l","r"}) {
    send(turn);
    for(int i=0;i<50 && (!pwmLeft || !pwmRight);++i) tick();
    assert(pwmLeft!=0 && pwmRight!=0);
    pins[sensor]=LOW; tick();
    assert(navigator.state==obstacle::State::RetryClearance); braked();
    pins[sensor]=HIGH;
    for(int i=0;i<200 && navigator.state!=obstacle::State::Idle;++i) tick();
    assert(navigator.state==obstacle::State::Idle); braked();
  }
  BLE.linked=true;tick();
  send("3");assert(navigator.selected==3);braked();
  send("f");assert(navigator.heading==0);braked();
  assert(navigator.state==obstacle::State::ScanRoute);
  for(int i=0;i<400 && navigator.state==obstacle::State::ScanRoute;++i) {
    tick(); braked(); assert(navigator.heading==0);
  }
  for(int i=0;i<400 && navigator.state!=obstacle::State::Drive;++i) tick();
  assert(navigator.route==obstacle::Route::Top && navigator.state==obstacle::State::Drive);
  assert(navigator.heading==3); // Left only after the route scan selected the top lane.
  // A finished side scan must not feed a stale side ray to the front controller.
  tick(40);assert(pwmLeft>0 && pwmRight>0);
  assert(pins[6]==LOW && pins[7]==HIGH && pins[9]==LOW && pins[8]==HIGH);
  send("s");braked();tick(20);braked();
  send("ml");assert(pwmLeft>0 && pwmRight==0);
  tick(25);braked();
  send("mr");assert(pwmLeft==0 && pwmRight>0);
  // Disconnect must stop immediately and stay stopped after reconnect.
  BLE.linked=false;tick();braked();assert(navigator.state==obstacle::State::Idle);
  BLE.linked=true;tick();braked();
  send("r");tick(2);assert(pwmLeft<0 && pwmRight>0);
  send("s");braked();
  // Stop arriving during a bounded sonar wait wins before the next motor command.
  send("ml");assert(pwmLeft>0);
  fakeEchoUs=0;fakeTime=lastPing+settings::PING_INTERVAL_MS;
  pingHook=[](){rx.incoming="s";};loop();braked();
  assert(!input.front.valid);
  // A backed-up USB output buffer must not block stop or motor checks.
  Serial.space=0;send("mr");assert(pwmRight>0);Serial.input="s";tick();braked();
  fakeEchoUs=11662; send("1f");
  for(int i=0;i<150 && scanIndex!=1;++i) tick();
  assert(scanIndex==1 && scanActive);
  send("s1f");
  assert(scanIndex==0 && servoAngle==settings::SERVO_LEFT);
  send("s");braked();
  send("r");
  for(int i=0;i<200 && navigator.state!=obstacle::State::Idle;++i) tick();
  assert(navigator.state==obstacle::State::Idle);braked();
  queueStatus();assert(strstr(report,"CONTROL=TIMED")!=nullptr);
  assert(strstr(report,"ENC_")==nullptr && strstr(report,"STALL")==nullptr);
  send("r");fakeEchoUs=300;fakeTime+=settings::PING_INTERVAL_MS;tick();
  assert(navigator.state==obstacle::State::RetryClearance);braked();
  BLE.linked=false;tick();braked();BLE.linked=true;fakeEchoUs=11662;fakeTime+=settings::PING_INTERVAL_MS;tick(100);braked();
  assert(navigator.state==obstacle::State::Idle);
  send("r");fakeEchoUs=300;fakeTime+=settings::PING_INTERVAL_MS;tick();
  assert(navigator.state==obstacle::State::RetryClearance);braked();
  send("s");fakeEchoUs=11662;fakeTime+=settings::PING_INTERVAL_MS;tick(100);braked();
  assert(navigator.state==obstacle::State::Idle);
  emergencyStop("TEST_RESET"); fakeEchoUs=0;
  for(int i=0;i<2;++i) { takePing(); assert(!input.front.valid && !input.front.noEcho); }
  takePing(); assert(!input.front.valid && input.front.noEcho);
  fakeEchoUs=100; takePing(); assert(!input.front.valid && !input.front.noEcho); // Too-short echo is not open space.
  fakeEchoUs=11662; takePing(); assert(input.front.valid && !input.front.noEcho);
  fakeEchoUs=0; send("1f");
  for(int i=0;i<400 && navigator.state!=obstacle::State::OpenDrive;++i) tick();
  assert(navigator.state==obstacle::State::OpenDrive);
  tick(100); assert(pwmLeft>0 && pwmRight>0);
  queueStatus(); assert(strstr(report,"FRONT_STATUS=NO_ECHO")!=nullptr);
  fakeEchoUs=1049; // Approximately 18 cm: actual obstacle must interrupt the open run.
  fakeTime=lastPing+settings::PING_INTERVAL_MS; loop();
  assert(navigator.state==obstacle::State::ScanAvoid);braked();
  send("s");fakeEchoUs=0;send("1f");
  for(int i=0;i<400 && navigator.state!=obstacle::State::OpenDrive;++i) tick();
  tick(50);assert(pwmLeft>0);BLE.linked=false;tick();braked();tick(100);braked();
  assert(navigator.state==obstacle::State::Idle);
  std::cout<<"PASS sketch integration: scan, pin polarity/brake, BLE/USB stop, disconnect, echo timeout, timed motion without wheel feedback\n";
}
