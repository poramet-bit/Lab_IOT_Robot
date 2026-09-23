#include <cassert>
#include <cstring>
#include <iostream>
#include "../Navigation.h"
using namespace obstacle;
Range range(float cm, uint32_t t = 0) { Range r; r.cm=cm; r.valid=true; r.at=t; return r; }
struct Rig {
  Navigator n; Input in; Output out;
  float front=200; bool valid=true, noEcho=false;
  uint32_t clockwiseMs=0, counterclockwiseMs=0;
  Rig() { in.front=range(200); in.left=range(200); in.right=range(200); }
  void step(uint32_t dt=50) {
    in.now += dt;
    if (out.left<0 && out.right>0) clockwiseMs+=dt;
    if (out.left>0 && out.right<0) counterclockwiseMs+=dt;
    in.front=range(front,in.now); in.front.valid=valid; in.front.noEcho=noEcho;
    out=n.update(in);
  }
  void choose(int die, Route route) {
    assert(n.select(die)); n.start(in);
    assert(n.state==State::ScanRoute && n.heading==0);
    chooseScannedRoute(route);
  }
  void chooseFacing(int heading, Route route) {
    assert(n.select(1)); n.start(in);
    // Inject a pose for forward/detour tests; real startup is tested separately.
    n.heading=heading;
    chooseScannedRoute(route);
  }
  void chooseScannedRoute(Route route) {
    int wanted=route==Route::Top ? 3 : 2;
    in.front=range(10); in.left=range(10); in.right=range(10);
    int relative=(wanted-n.heading+4)%4;
    if(relative==0) in.front=range(200);
    if(relative==1) in.right=range(200);
    if(relative==3) in.left=range(200);
    ++in.scanId; out=n.update(in); assert(n.route==route);
  }
  void reach(State s, int limit=500) {
    while(n.state!=s && n.state!=State::Fault && limit-->0) step();
    if(n.state!=s) std::cerr<<"Expected "<<stateName(s)<<", got "<<stateName(n.state)<<" "<<n.reason<<"\n";
    assert(n.state==s);
  }
};
void routes() {
  for(int die=1;die<=6;++die) for(Route route : {Route::Top,Route::Bottom}) {
    Rig r; r.choose(die,route);
    assert(!r.n.select(1)); // Cannot silently change an active mission.
    r.reach(State::Drive); r.step();
    int travel=route==Route::Top ? 3 : 2;
    bool reverse=route==Route::Bottom;
    assert(r.n.heading==(reverse ? 0 : travel));
    assert(reverse ? (r.out.left<0 && r.out.right<0) : (r.out.left>0 && r.out.right>0));
    r.reach(State::ExitDrive,1800);
    assert(r.n.heading==2 && r.n.x<50 && r.n.x>20);
    for(int i=0;i<20;++i) { r.step(); assert(r.out.left>0 && r.out.right>0); }
    r.n.stop(); r.step(); assert(r.out.left==0 && r.out.right==0);
  }
  Rig r; assert(!r.n.select(0) && !r.n.select(7)); r.n.start(r.in); assert(!r.n.active());
  assert(r.n.select(3)); r.n.start(r.in); r.reach(State::ScanRoute); ++r.in.scanId; r.n.update(r.in);
  assert(r.n.route==Route::Top); // From north, a visible left route beats blind reverse.
  std::cout<<"PASS all dice / both routes, relative turns, exit continues\n";
}
void turn_and_failures() {
  for(int side : {-1,1}) {
    Rig r; assert(r.n.test90(side,r.in)); r.step();
    assert(r.out.left*side<0 && r.out.right*side>0);
    r.reach(State::Idle);
    uint32_t onMs=side<0?r.counterclockwiseMs:r.clockwiseMs;
    uint32_t target=side<0?settings::TURN_LEFT_90_MS:settings::TURN_RIGHT_90_MS;
    assert(onMs>=target && onMs<target+50);
  }
  Rig r; r.chooseFacing(2,Route::Bottom); r.reach(State::Drive); r.step();
  r.valid=false; r.step(); assert(r.out.left==0 && r.out.right==0);
  r.reach(State::Fault,40); assert(!std::strcmp(r.n.reason,"FRONT_RANGE_INVALID"));
  Rig gap; gap.chooseFacing(2,Route::Bottom); gap.step(settings::CONTROL_GAP_MS+1);
  assert(gap.n.state==State::Fault && !std::strcmp(gap.n.reason,"CONTROL_GAP"));
  Rig scan; scan.n.select(3); scan.n.start(scan.in); scan.reach(State::ScanRoute); scan.reach(State::Fault,110);
  assert(!std::strcmp(scan.n.reason,"SCAN_TIMEOUT"));
  Rig noEcho; noEcho.n.select(1); noEcho.n.start(noEcho.in);
  noEcho.in.front.valid=noEcho.in.left.valid=noEcho.in.right.valid=false;
  ++noEcho.in.scanId; noEcho.n.update(noEcho.in);
  assert(noEcho.n.state==State::Fault); // A failed sonar must not select blind reverse.
  Rig wrap; wrap.in.now=UINT32_MAX-100; wrap.n.select(1); wrap.n.start(wrap.in);
  for(int i=0;i<10;++i) wrap.step();
  assert(wrap.n.state==State::ScanRoute);
  for(State s : {State::ScanRoute,State::Brake,State::Align,State::Turn,State::Drive,State::OpenDrive,State::ClearRear,State::ScanAvoid,
                State::Skirt,State::ReturnLane,State::ExitDrive,State::TestWheel,
                State::RetryClearance,State::Backoff}) {
    Rig stop; stop.n.state=s; stop.n.stop();
    for(int i=0;i<5;++i) { stop.step(); assert(stop.n.state==State::Idle && !stop.out.left && !stop.out.right); }
  }
  std::cout<<"PASS timed turns, faults, clock wrap, stop latch\n";
}
void detour() {
  Rig r; r.chooseFacing(2,Route::Bottom); r.reach(State::Drive); r.step();
  r.front=18; r.step(); assert(r.n.state==State::ScanAvoid && r.out.left==0);
  r.in.left=range(10); r.in.right=range(200); ++r.in.scanId; r.step();
  r.front=200; r.reach(State::Skirt); assert(r.n.heading==3);
  // No previous edge detection must never count as already cleared.
  for(int i=0;i<12;++i) { r.step(); assert(r.n.state==State::Skirt); }
  r.in.irFrontLeft=true; for(int i=0;i<5;++i) r.step();
  r.in.irFrontLeft=false;
  const float skirtCmPerSecond=settings::FORWARD_CM_PER_SEC*settings::SLOW_PWM/settings::RUN_PWM;
  const uint32_t halfClearMs=uint32_t(500.0f*settings::BODY_CLEAR_CM/skirtCmPerSecond);
  for(uint32_t t=0;t<halfClearMs;t+=50) { r.step(); assert(r.n.state==State::Skirt); }
  // Wait for the full body to clear, then turn back along the route.
  r.reach(State::Brake); r.reach(State::Skirt); assert(r.n.heading==2);
  r.in.irFrontLeft=true; for(int i=0;i<8;++i) r.step();
  r.in.irFrontLeft=false; r.reach(State::ReturnLane); assert(r.n.heading==1);
  r.reach(State::Drive); assert(r.n.heading==2 && r.n.leg==0);
  assert(r.n.x>240 && r.n.x<260);
  Rig blocked; blocked.chooseFacing(2,Route::Bottom); blocked.reach(State::Drive); blocked.front=18;
  blocked.step(); blocked.in.left=range(10); blocked.in.right=range(10); ++blocked.in.scanId;
  blocked.step(); assert(blocked.n.state==State::RetryClearance);
  std::cout<<"PASS bounded edge following, body clearance, lane return, blocked detour\n";
}
void rear_wall() {
  Rig r; r.choose(1,Route::Bottom); r.reach(State::Drive); r.step();
  r.n.y=60; r.in.irBackLeft=true; r.step();
  // Rear IR wall detection must release the wall before rotating a long body.
  for(int i=0;i<10;++i) { r.step(); assert(r.n.state!=State::Fault); }
  assert(r.out.left>0 && r.out.right>0);
  r.in.irBackLeft=false; r.reach(State::Turn); assert(r.n.leg==1);
  r.reach(State::Drive); assert(r.n.heading==3);
  std::cout<<"PASS rear wall clearance before turning\n";
}
void rear_corner_guards() {
  for(bool Input::*sensor : {&Input::irBackLeft, &Input::irBackRight}) {
    Rig r; r.choose(1,Route::Bottom); r.reach(State::Drive); r.step();
    assert(r.out.left<0 && r.out.right<0);
    r.n.y=60; r.in.*sensor=true; r.step();
    assert(!r.out.left && !r.out.right);
    r.reach(State::ClearRear);
    // Keep the rear corner occupied beyond the minimum release distance.
    for(int i=0;i<10;++i) { r.step(); assert(r.n.state==State::ClearRear); }
    assert(r.out.left>0 && r.out.right>0);
    r.in.*sensor=false;
    r.reach(State::Drive); assert(r.n.heading==3 && r.n.leg==1);

    Rig backing; backing.chooseFacing(2,Route::Bottom); backing.reach(State::Drive);
    backing.front=18; backing.step();
    backing.in.left=range(10); backing.in.right=range(10); ++backing.in.scanId; backing.step();
    backing.reach(State::ScanAvoid); ++backing.in.scanId; backing.step();
    backing.reach(State::Backoff); backing.step(); assert(backing.out.left<0);
    backing.in.*sensor=true; backing.step();
    assert(!backing.out.left && !backing.out.right);
    backing.reach(State::ScanAvoid);

    Rig blocked; blocked.n.select(1); blocked.n.start(blocked.in);
    blocked.in.*sensor=true;
    blocked.in.front=blocked.in.left=blocked.in.right=range(10);
    ++blocked.in.scanId; blocked.out=blocked.n.update(blocked.in);
    assert(blocked.n.state==State::RetryClearance);
    assert(!std::strcmp(blocked.n.lastIssue,"NO_START_CLEARANCE"));
  }
  std::cout<<"PASS either rear corner stops reverse/backoff and guards route selection/release\n";
}
void corner_edge_following() {
  for(int side : {-1,1}) {
    Rig r; r.chooseFacing(2,Route::Bottom); r.reach(State::Drive);
    r.n.x=150; r.n.y=150; // Room for either side of the obstacle.
    r.front=18; r.step();
    r.in.left=range(side<0 ? 200 : 10); r.in.right=range(side>0 ? 200 : 10);
    ++r.in.scanId; r.step(); r.front=200; r.reach(State::Skirt);
    bool& frontEdge=side<0 ? r.in.irFrontRight : r.in.irFrontLeft;
    bool& rearEdge=side<0 ? r.in.irBackRight : r.in.irBackLeft;
    frontEdge=rearEdge=true; r.step(); frontEdge=false;
    // The front has passed, but the rear still sees the same obstacle.
    for(int i=0;i<20;++i) { r.step(); assert(r.n.state==State::Skirt); }
    rearEdge=false;
    r.reach(State::Brake); r.reach(State::Skirt); assert(r.n.heading==2);
    bool& oppositeRear=side<0 ? r.in.irBackLeft : r.in.irBackRight;
    oppositeRear=true; r.step();
    assert(r.n.state==State::RetryClearance && !r.out.left && !r.out.right);
    assert(!std::strcmp(r.n.lastIssue,"DETOUR_SIDE_BLOCKED"));
  }
  std::cout<<"PASS both corner sensors must clear an edge; opposite rear corner blocks detour\n";
}
void front_corner_steering() {
  Rig r; r.chooseFacing(2,Route::Bottom); r.reach(State::Drive); r.step();
  r.in.irBackLeft=r.in.irBackRight=true; r.step();
  assert(r.out.left>0 && r.out.left==r.out.right); // Forward travel can release a rear obstacle.
  r.in.irBackLeft=r.in.irBackRight=false;
  r.in.irFrontLeft=true; r.step(); assert(r.out.left>r.out.right && r.out.right>0);
  r.in.irFrontLeft=false; r.in.irFrontRight=true; r.step();
  assert(r.out.right>r.out.left && r.out.left>0);
  r.in.irFrontLeft=true; r.step();
  assert(r.n.state==State::RetryClearance && !r.out.left && !r.out.right);
  std::cout<<"PASS front corners steer away independently and brake when both detect\n";
}
void clearance_recovery() {
  Rig r; r.chooseFacing(2,Route::Bottom); r.reach(State::Drive); r.front=18; r.step();
  r.in.left=range(10); r.in.right=range(10); ++r.in.scanId; r.step();
  assert(r.n.state==State::RetryClearance && !r.out.left && !r.out.right);
  r.reach(State::ScanAvoid); r.step();
  assert(r.n.state==State::ScanAvoid); // Must wait for new scan data, not reuse the failed scan.
  ++r.in.scanId; r.step(); // A second blocked scan.
  r.reach(State::Backoff); float before=r.n.y; r.step();
  assert(r.out.left<0 && r.out.right<0);
  r.reach(State::ScanAvoid); assert(r.n.y-before>=settings::RECOVERY_BACKOFF_CM);
  r.in.right=range(200); ++r.in.scanId; r.step(); r.front=200;
  r.reach(State::Skirt); assert(r.n.heading==3 && r.n.leg==0);

  Rig blocked; blocked.chooseFacing(2,Route::Bottom); blocked.reach(State::Drive);
  blocked.front=18; blocked.step(); blocked.in.irBackLeft=true;
  blocked.in.left=range(10); blocked.in.right=range(10);
  for(int i=0;i<200 && blocked.n.state!=State::Fault;++i) {
    if(blocked.n.scanning()) ++blocked.in.scanId;
    blocked.step(); assert(!blocked.out.left && !blocked.out.right);
  }
  assert(blocked.n.state==State::Fault && blocked.n.clearanceRetries==settings::MAX_CLEARANCE_RETRIES);

  Rig rear; rear.chooseFacing(2,Route::Bottom); rear.reach(State::Drive); rear.front=18; rear.step();
  rear.in.left=range(10); rear.in.right=range(10); ++rear.in.scanId; rear.step();
  rear.reach(State::ScanAvoid); ++rear.in.scanId; rear.step(); rear.reach(State::Backoff);
  rear.step(); assert(rear.out.left<0); rear.in.irBackLeft=true; rear.step();
  assert(rear.out.left==0 && rear.out.right==0); rear.reach(State::ScanAvoid);

  Rig start; start.n.select(3); start.n.start(start.in); start.reach(State::ScanRoute);
  start.in.irBackRight=true; // From zero, the rear route must also be blocked.
  start.in.front=start.in.left=start.in.right=range(10,start.in.now);
  ++start.in.scanId; start.out=start.n.update(start.in);
  assert(start.n.state==State::RetryClearance && !std::strcmp(start.n.lastIssue,"NO_START_CLEARANCE"));
  start.reach(State::ScanRoute); start.in.irBackRight=false;
  start.in.left=range(200,start.in.now); ++start.in.scanId;
  start.n.update(start.in); start.reach(State::Drive); assert(start.n.route==Route::Top);

  Rig turn; turn.n.test90(1,turn.in); for(int i=0;i<5;++i) turn.step();
  turn.front=settings::TURN_STOP_CM - 1; turn.step(); assert(turn.n.state==State::RetryClearance);
  turn.front=200; turn.reach(State::Idle,100);
  assert(turn.clockwiseMs>=settings::TURN_RIGHT_90_MS && turn.clockwiseMs<settings::TURN_RIGHT_90_MS+50);
  std::cout<<"PASS blocked scan retries/backoff, rear guard, recovery budget, partial-turn resume\n";
}
void same_heading_startup() {
  for(int die=1;die<=6;++die) {
    Rig r; assert(r.n.select(die)); r.in.scanId=7; r.n.start(r.in);
    assert(r.n.state==State::ScanRoute && r.n.heading==0);
    // Selection starts a fresh scan; waiting for it must not energize the motors.
    for(int i=0;i<20;++i) {
      r.step();
      assert(r.n.state==State::ScanRoute && r.n.heading==0);
      assert(!r.out.left && !r.out.right);
    }
    assert(r.clockwiseMs==0 && r.counterclockwiseMs==0);
    assert(r.n.x==251.5f && r.n.y==245.f);
    r.n.stop(); assert(r.n.test90(1,r.in)); r.reach(State::Idle);
    assert(r.n.heading==1);
    r.n.start(r.in); // A fresh run assumes physical placement at zero again.
    assert(r.n.state==State::ScanRoute && r.n.heading==0 && r.n.selected==die);
  }
  std::cout<<"PASS all six scenarios scan immediately from zero without startup rotation\n";
}
void no_encoder_turn_regression() {
  Navigator n; Input in; in.front=range(200);
  assert(n.test90(1,in));
  for(int i=0;i<400 && n.active();++i) {
    in.now+=20; in.front=range(200,in.now); n.update(in);
  }
  assert(n.state==State::Idle && n.heading==1);
  std::cout<<"PASS turn finishes without any wheel pulses\n";
}
void timed_pause_and_wrap() {
  Rig r; r.n.test90(1,r.in); for(int i=0;i<4;++i) r.step();
  r.valid=false; r.step(); uint32_t before=r.clockwiseMs;
  for(int i=0;i<10;++i) { r.step(); assert(!r.out.left && !r.out.right); }
  assert(r.clockwiseMs==before);
  r.valid=true; r.reach(State::Idle);
  assert(r.clockwiseMs>=settings::TURN_RIGHT_90_MS && r.clockwiseMs<settings::TURN_RIGHT_90_MS+50);
  Rig wrap; wrap.in.now=UINT32_MAX-100; wrap.in.front.at=wrap.in.now;
  wrap.n.test90(-1,wrap.in); wrap.reach(State::Idle);
  assert(wrap.n.heading==3 && wrap.counterclockwiseMs>=settings::TURN_LEFT_90_MS);
  std::cout<<"PASS sensor pauses excluded from motor time; timer rollover\n";
}
void no_echo_forward() {
  Rig r; r.chooseFacing(2,Route::Bottom); r.reach(State::Drive);
  r.valid=false; r.noEcho=true; r.front=0;
  const uint32_t start=r.in.now;
  while(r.in.now-start<settings::LEG_TIMEOUT_MS+2000) {
    r.step(); assert(r.out.left>0 && r.out.right>0);
  }
  assert(r.n.heading==2 && r.n.leg==0); // No timed corner or travel deadline in open space.
  r.valid=true; r.noEcho=false; r.front=150; r.step(); assert(r.out.left>0);
  r.front=18; r.step(); assert(r.n.state==State::ScanAvoid && !r.out.left && !r.out.right);
  r.in.left=range(10); r.in.right=range(200); ++r.in.scanId; r.step(); r.front=200;
  r.reach(State::Skirt);
  r.in.irFrontLeft=true; for(int i=0;i<5;++i) r.step(); r.in.irFrontLeft=false;
  r.reach(State::Brake); r.reach(State::Skirt);
  r.in.irFrontLeft=true; for(int i=0;i<5;++i) r.step(); r.in.irFrontLeft=false;
  r.reach(State::ReturnLane); r.reach(State::OpenDrive); r.step();
  assert(r.n.heading==2 && r.out.left>0 && r.out.right>0);
  r.n.stop(); r.step(); assert(!r.out.left && !r.out.right);

  Rig startOpen; startOpen.n.select(1); startOpen.n.start(startOpen.in);
  startOpen.valid=false; startOpen.noEcho=true; startOpen.front=0;
  startOpen.in.left.valid=startOpen.in.right.valid=false;
  ++startOpen.in.scanId; startOpen.step(); startOpen.step();
  assert(startOpen.out.left>0 && startOpen.out.right>0 && startOpen.n.heading==0);
  startOpen.in.irFrontLeft=startOpen.in.irFrontRight=true; startOpen.step();
  assert(!startOpen.out.left && !startOpen.out.right);

  Rig backing; backing.choose(1,Route::Bottom); backing.reach(State::Drive); backing.step();
  assert(backing.out.left<0); backing.noEcho=true; backing.valid=false; backing.front=0;
  backing.step(); assert(backing.n.state==State::Brake && !backing.out.left && !backing.out.right);
  backing.reach(State::OpenDrive); backing.step(); assert(backing.out.left>0 && backing.out.right>0);

  Rig stale; stale.n.select(1); stale.n.start(stale.in); stale.in.front.valid=false;
  stale.in.front.noEcho=true; ++stale.in.scanId; stale.n.update(stale.in);
  for(int i=0;i<40 && stale.n.state!=State::Fault;++i) {
    stale.in.now+=50; stale.out=stale.n.update(stale.in); // No timestamp refresh.
    if(stale.in.now>settings::RANGE_MAX_AGE_MS) assert(!stale.out.left && !stale.out.right);
  }
  assert(stale.n.state==State::Fault && !std::strcmp(stale.n.reason,"FRONT_RANGE_INVALID"));
  std::cout<<"PASS confirmed no echo drives continuously; obstacle, IR and stop still interrupt\n";
}
int main() { same_heading_startup(); rear_corner_guards(); corner_edge_following(); front_corner_steering(); no_echo_forward(); no_encoder_turn_regression(); timed_pause_and_wrap(); clearance_recovery(); routes(); turn_and_failures(); detour(); rear_wall(); }
