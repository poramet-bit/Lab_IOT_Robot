#include <iostream>
#include <Servo.h>
#include "../obstacle_avoidance_no_servo.ino"

unsigned long frontEchoUs=11662, rightEchoUs=11662, leftEchoUs=11662;
int rightPings=0, leftPings=0, centeredPings=0;
int backupStarts=0, backupFinishes=0, scanStarts=0;
uint32_t backupStartedAt=0;
uint32_t pingStartedAt=0, lastCenterMove=0;

void braked() {
  assert(pwmLeft==0 && pwmRight==0);
  assert(pwm[5]==255 && pwm[10]==255);
  assert(pins[6]==HIGH && pins[7]==HIGH && pins[9]==HIGH && pins[8]==HIGH);
}
void observePing() {
  if(fakeServoPin<0) return; // Lets the pre-servo sketch fail at the wiring assertion.
  assert(pingStartedAt-fakeServoMovedAt>=300); // Never measure a moving servo.
  if(fakeServoAngle==30) { ++rightPings; braked(); }
  else if(fakeServoAngle==150) { ++leftPings; braked(); }
  else { assert(fakeServoAngle==90); ++centeredPings; }
}
void tick() {
  const bool wasBacking=pwmLeft<0 && pwmRight<0;
  const int previousServoAngle=fakeServoAngle;
  fakeEchoUs=fakeServoAngle==30 ? rightEchoUs : (fakeServoAngle==150 ? leftEchoUs : frontEchoUs);
  fakeTime+=10;
  pingStartedAt=millis();
  if(fakeServoAngle==90 && lastCenterMove!=fakeServoMovedAt) {
    lastCenterMove=fakeServoMovedAt; centeredPings=0;
  }
  pingHook=observePing;
  loop();
  const bool backing=pwmLeft<0 && pwmRight<0;
  if(backing && !wasBacking) { ++backupStarts; backupStartedAt=millis(); }
  if(wasBacking && !backing) {
    ++backupFinishes;
    // 180 ms command, allowing a control tick and the bounded sonar wait.
    assert(millis()-backupStartedAt>=140 && millis()-backupStartedAt<=220);
  }
  if(fakeServoAngle==30 && previousServoAngle!=30) {
    ++scanStarts;
    assert(backupFinishes==scanStarts && "Every new sweep needs exactly one completed retreat");
  }
  if(fakeServoAngle!=90 || millis()-fakeServoMovedAt<300) braked();
}
void advance(uint32_t ms) {
  const uint32_t began=millis();
  while(millis()-began<ms) tick();
}
void startClear() {
  frontEchoUs=rightEchoUs=leftEchoUs=11662; // 200 cm in every direction.
  rightPings=leftPings=centeredPings=0;
  backupStarts=backupFinishes=scanStarts=0;
  setup(); advance(3200);
  assert(pwmLeft>0 && pwmRight>0);
}
void waitForMotion(int leftSign, int rightSign) {
  const uint32_t began=millis();
  while(millis()-began<4000) {
    tick();
    if(pwmLeft*leftSign>0 && pwmRight*rightSign>0) return;
  }
  assert(false && "Expected motor direction was never reached");
}
void waitForTurn(bool expectedRight) {
  const uint32_t began=millis();
  while(millis()-began<4000) {
    tick();
    if((pwmLeft>0 && pwmRight<0) || (pwmLeft<0 && pwmRight>0)) {
      assert(turnRight==expectedRight);
      // Max_Speed maps logical wheels to the opposite motor channels.
      assert(pwmLeft==(expectedRight ? -160 : 160));
      assert(pwmRight==(expectedRight ? 134 : -134));
      return;
    }
  }
  assert(false && "Expected turn was never reached");
}
void finishBackup() {
  waitForMotion(-1,-1);
  assert(pins[6]==HIGH && pins[7]==LOW && pins[9]==HIGH && pins[8]==LOW);
  const uint32_t began=millis();
  while(pwmLeft<0 && pwmRight<0 && millis()-began<1000) tick();
  assert(millis()-began<1000); // A short retreat must end even while the obstacle remains.
  braked();
}
void advanceBlocked(uint32_t ms) {
  const uint32_t began=millis();
  while(millis()-began<ms) {
    tick();
    if(pwmLeft==0 && pwmRight==0) braked();
    else assert(pwmLeft<0 && pwmRight<0); // Retreats allowed; never turn or drive into the corner.
  }
}
void startup_and_wiring() {
  modes[A5]=modes[2]=modes[3]=modes[11]=-1;
  const uint32_t began=millis();
  setup();
  assert(fakeServoPin==A2 && fakeServoAngle==90);
  while(millis()-began<2900) { tick(); braked(); }
  advance(200);
  assert(backupStarts==0 && scanStarts==0); // Startup centering is not an avoidance sweep.
  assert(pwmBits==8 && pwmLeft==180 && pwmRight==151);
  assert(pins[6]==LOW && pins[7]==HIGH && pins[9]==LOW && pins[8]==HIGH);
  assert(modes[A3]==INPUT_PULLUP && modes[A4]==INPUT_PULLUP);
  assert(modes[A5]==-1 && modes[2]==-1 && modes[3]==-1 && modes[11]==-1);
  std::cout<<"PASS servo on A2, automatic 3-second start, Max_Speed motor PWM and trim\n";
}
void motor_calibration_matches_max_speed() {
  startClear();
  assert(pwm[5]==180 && pwm[10]==151);
  drive(0,0); braked();
  drive(160,-160);
  assert(pwmLeft==-160 && pwmRight==134 && pwm[5]==160 && pwm[10]==134);
  drive(0,0); braked();
  drive(-130,-130);
  assert(pwmLeft==-130 && pwmRight==-109 && pwm[5]==130 && pwm[10]==109);
  drive(0,0); braked();
  std::cout<<"PASS Max_Speed brake, forward, reverse and swapped turn channels\n";
}
void obstacle_backs_up_before_right_turn() {
  startClear(); frontEchoUs=1049; // 18 cm ahead; sides are clear.
  finishBackup();
  waitForTurn(true);
  assert(backupStarts==1 && backupFinishes==1 && scanStarts==1);
  assert(rightPings>=3 && leftPings>=3);
  assert(fakeServoAngle==90 && centeredPings>0);
  assert(pins[6]==HIGH && pins[7]==LOW && pins[9]==LOW && pins[8]==HIGH);
  frontEchoUs=11662;
  waitForMotion(1,1);
  std::cout<<"PASS backup -> stationary side scan -> fresh front reading -> right turn -> forward\n";
}
void blocked_right_sonar_chooses_left() {
  startClear(); frontEchoUs=rightEchoUs=1049; // IRs alone cannot see this obstacle.
  finishBackup(); waitForTurn(false);
  assert(rightPings>=3 && leftPings>=3);
  assert(fakeServoAngle==90);
  std::cout<<"PASS side ultrasonic vetoes the preferred right turn\n";
}
void clearer_side_wins_with_right_tie_preference() {
  struct Choice { unsigned long leftUs, rightUs; bool right, leftIr, rightIr; };
  const Choice choices[] = {
    {11662, 3499, false, false, false}, // Left 200 cm, right 60 cm: choose more room.
    {3499, 11662, true, false, false},  // Right 200 cm, left 60 cm.
    {4082, 3499, false, false, false},  // Left 70 cm, right 60 cm: farther always wins.
    {3500, 3499, false, false, false},  // Even a small measured advantage selects left.
    {3499, 3500, true, false, false},
    {3499, 3499, true, false, false},   // Only an exact side tie prefers right.
    {22741, 21865, false, false, false}, // 390 vs 375 cm: scans retain their full range.
    {21865, 22741, true, false, false},
    {11662, 0, false, false, false},   // Measured clear left beats unmeasured right.
    {0, 11662, true, false, false},    // Measured clear right beats unmeasured left.
    {11662, 3499, true, true, false},  // The roomier side is vetoed by its IR.
    {3499, 11662, false, false, true}
  };
  for(const Choice& choice:choices) {
    startClear(); frontEchoUs=1049;
    leftEchoUs=choice.leftUs; rightEchoUs=choice.rightUs;
    pins[A4]=choice.leftIr ? LOW : HIGH;
    pins[A3]=choice.rightIr ? LOW : HIGH;
    finishBackup();
    const uint32_t began=millis();
    while(millis()-began<4000) {
      tick();
      if((pwmLeft>0 && pwmRight<0) || (pwmLeft<0 && pwmRight>0)) break;
      braked(); // Never drive forward or repeat the retreat while choosing.
    }
    assert(millis()-began<4000);
    assert(turnRight==choice.right);
    assert((pwmLeft<0)==choice.right);
    assert((pwmRight>0)==choice.right);
    assert(fakeServoAngle==90);
  }
  std::cout<<"PASS farthest measured side through 390 cm, exact ties, IR veto and no-echo fallback\n";
}
void front_ir_steers_away() {
  startClear(); pins[A4]=LOW;
  finishBackup(); waitForTurn(true); // Object on left: turn right.
  startClear(); pins[A3]=LOW;
  finishBackup(); waitForTurn(false); // Object on right: turn left.
  assert(pins[6]==LOW && pins[7]==HIGH && pins[9]==HIGH && pins[8]==LOW);
  std::cout<<"PASS both front IRs trigger a retreat and choose a clear turn\n";
}
void corner_retries_back_up_before_each_scan() {
  startClear(); pins[A3]=pins[A4]=LOW;
  finishBackup();
  advanceBlocked(4000);
  assert(backupStarts>=3 && scanStarts>=3);
  pins[A3]=HIGH; waitForTurn(true); // No reset or command is required.

  startClear(); frontEchoUs=400; // About 7 cm, too close even after the retreat.
  finishBackup();
  advanceBlocked(4000);
  assert(backupStarts>=3 && scanStarts>=3);
  frontEchoUs=1049; waitForTurn(true);
  std::cout<<"PASS corner IRs or very close front -> short retreat before every rescan -> recover\n";
}
void blocked_sides_retreat_before_rescanning() {
  startClear(); frontEchoUs=rightEchoUs=leftEchoUs=1049;
  finishBackup();
  advanceBlocked(4500);
  assert(backupStarts>=3 && scanStarts>=3);
  assert(rightPings>=6 && leftPings>=6); // Re-scan, rather than waiting on stale side data.
  rightEchoUs=11662; waitForTurn(true);
  std::cout<<"PASS both sides blocked -> repeated bounded retreats and stationary scans -> clear turn\n";
}
void turn_guards_and_unused_rear() {
  pins[A5]=pins[2]=LOW; // Former rear sensors have no influence.
  startClear(); frontEchoUs=1049;
  finishBackup(); waitForTurn(true);
  pins[A3]=LOW; tick(); braked();
  finishBackup(); // An interrupted turn must also retreat before moving the servo.
  waitForTurn(false);
  pins[A4]=LOW; tick(); braked();
  advanceBlocked(1500);
  pins[A3]=pins[A4]=HIGH; frontEchoUs=1049;
  waitForTurn(true); frontEchoUs=11662; waitForMotion(1,1);
  std::cout<<"PASS IR interrupts turning; recovery and unused rear inputs\n";
}
void sonar_and_clock_wrap() {
  startClear(); frontEchoUs=rightEchoUs=leftEchoUs=0; advance(500);
  assert(pwmLeft>0 && pwmRight>0); // Existing three-timeout open-space policy.
  pins[A4]=LOW; finishBackup(); waitForTurn(true);
  assert(rightPings>=3 && leftPings>=3);

  startClear(); frontEchoUs=100; advance(1700); braked(); // Echo too short is invalid.
  frontEchoUs=11662; waitForMotion(1,1); // Recover without a fault reset.

  fakeTime=UINT32_MAX-1500; frontEchoUs=11662;
  setup(); advance(2800); braked(); advance(400);
  assert(pwmLeft>0 && pwmRight>0);
  fakeTime=UINT32_MAX-100; frontEchoUs=1049; // Retreat and brakes cross timer rollover.
  finishBackup(); waitForTurn(true);
  std::cout<<"PASS no echo, invalid echo recovery and timer rollover\n";
}
void side_readings_cannot_substitute_for_front() {
  startClear(); frontEchoUs=1049;
  finishBackup(); frontEchoUs=100; // Invalid front even though both scanned sides are clear.
  advanceBlocked(3000);
  frontEchoUs=0;
  advanceBlocked(170); // One or two front timeouts must not authorize a turn.
  waitForTurn(true);
  assert(centeredPings>=3);
  std::cout<<"PASS side scan data cannot authorize a turn without a fresh front reading\n";
}
void invalid_side_echo_is_not_open_space() {
  startClear(); frontEchoUs=1049; rightEchoUs=100; leftEchoUs=1049;
  finishBackup();
  advanceBlocked(3000);
  rightEchoUs=11662; waitForTurn(true);
  std::cout<<"PASS invalid side echoes block a turn until a fresh clear scan\n";
}
void forward_stops_at_25_cm() {
  startClear();
  frontEchoUs=1459; // 25.02 cm: just beyond the stop threshold.
  advance(200);
  assert(pwmLeft>0 && pwmRight>0 && backupStarts==0);
  frontEchoUs=1457; // 24.99 cm: brake before starting the retreat.
  advance(100); braked();
  assert(mode==Mode::Backup && backupStarts==0);
  finishBackup();
  std::cout<<"PASS drive above 25 cm, brake at 25 cm, then one short retreat\n";
}
void farthest_route_includes_fresh_front() {
  struct Choice { unsigned long frontUs, leftUs, rightUs; bool forward, right; };
  const Choice choices[] = {
    {17493, 11662, 14577, true, false}, // Front 300 cm beats left 200 and right 250.
    {11662, 11662, 11662, true, false}, // An exact tie can continue straight.
    {11662, 1049, 1049, true, false},   // Front is open even though both sides are blocked.
    {3499, 4082, 2915, false, false},   // Left 70 > front 60 > right 50.
    {3499, 2915, 4082, false, true},
    {0, 11662, 3499, false, false}     // A measured side beats an unmeasured front.
  };
  for(const Choice& choice:choices) {
    startClear(); frontEchoUs=1049;
    leftEchoUs=choice.leftUs; rightEchoUs=choice.rightUs;
    finishBackup();
    frontEchoUs=choice.frontUs; // New front clearance after the retreat.
    const uint32_t began=millis();
    while(pwmLeft==0 && pwmRight==0 && millis()-began<4000) tick();
    assert(millis()-began<4000);
    assert(backupStarts==1 && rightPings>=3 && leftPings>=3 && centeredPings>0);
    assert(fakeServoAngle==90);
    if(choice.forward) assert(pwmLeft>0 && pwmRight>0);
    else {
      assert(turnRight==choice.right);
      assert(pwmLeft==(choice.right ? -160 : 160));
      assert(pwmRight==(choice.right ? 134 : -134));
    }
  }
  std::cout<<"PASS compare fresh front with both side ranges and follow the farthest clear route\n";
}
void calibrated_turn_duration() {
  for(int right=0; right<2; ++right) {
    startClear(); frontEchoUs=1049;
    if(!right) rightEchoUs=1049;
    finishBackup(); waitForTurn(right!=0);
    const uint32_t began=millis();
    frontEchoUs=11662;
    while(pwmLeft!=0 && pwmRight!=0 && millis()-began<1000) tick();
    assert(millis()-began>=450 && millis()-began<=480);
    braked();
    assert(mode==Mode::Forward);
  }
  std::cout<<"PASS both turns use the Max_Speed 450 ms timing before braking\n";
}
int main(int argc, char** argv) {
  struct Test { const char* name; void (*run)(); };
  const Test tests[] = {
    {"startup", startup_and_wiring},
    {"motors", motor_calibration_matches_max_speed},
    {"threshold", forward_stops_at_25_cm},
    {"right_turn", obstacle_backs_up_before_right_turn},
    {"ir_sides", front_ir_steers_away},
    {"farthest_side", clearer_side_wins_with_right_tie_preference},
    {"farthest_route", farthest_route_includes_fresh_front},
    {"turn_duration", calibrated_turn_duration},
    {"blocked_right", blocked_right_sonar_chooses_left},
    {"blocked_sides", blocked_sides_retreat_before_rescanning},
    {"corner", corner_retries_back_up_before_each_scan},
    {"turn_guards", turn_guards_and_unused_rear},
    {"sonar_clock", sonar_and_clock_wrap},
    {"fresh_front", side_readings_cannot_substitute_for_front},
    {"invalid_side", invalid_side_echo_is_not_open_space}
  };
  int passed=0;
  for(const Test& test:tests) {
    if(argc>1 && std::strcmp(argv[1],test.name)!=0) continue;
    std::cout<<"RUN "<<test.name<<std::endl;
    test.run(); ++passed;
  }
  assert(passed>0);
  std::cout<<passed<<" tests passed\n";
}
