#pragma once
#include "Settings.h"

namespace obstacle {
struct Range { float cm = 0; bool valid = false; uint32_t at = 0; bool noEcho = false; };
struct Input {
  uint32_t now = 0, scanId = 0;
  Range front, left, right;
  bool irFrontLeft = false, irFrontRight = false;
  bool irBackLeft = false, irBackRight = false;
  bool rearBlocked() const { return irBackLeft || irBackRight; }
  bool leftBlocked() const { return irFrontLeft || irBackLeft; }
  bool rightBlocked() const { return irFrontRight || irBackRight; }
  bool anyIrBlocked() const { return leftBlocked() || rightBlocked(); }
};
struct Output { int left = 0, right = 0; };
enum class State { Idle, ScanRoute, Align, Turn, Brake, Drive, OpenDrive, ClearRear, ScanAvoid, Skirt, ReturnLane, ExitDrive, TestWheel, RetryClearance, Backoff, Fault };
enum class Route { Top, Bottom };

// Hardware-independent controller. Headings: N=0, E=1, S=2, W=3.
// Motion is timed. x/y are coarse estimates from commanded PWM, not measured position.
class Navigator {
public:
  State state = State::Idle;
  Route route = Route::Top;
  int heading = 0, selected = 0, leg = 0;
  int clearanceRetries = 0;
  float x = 0, y = 0;
  const char* reason = "READY";
  const char* lastIssue = "NONE";

  bool active() const { return state != State::Idle && state != State::Fault; }
  bool scanning() const { return state == State::ScanRoute || state == State::ScanAvoid; }
  bool select(int n) {
    if (active() || n < 1 || n > 6) return false;
    selected = n;
    return true;
  }
  void stop(const char* why = "STOP") {
    state = State::Idle; reason = why; previous = Output();
  }
  void start(const Input& in) {
    if (active()) return;
    if (!selected) { stop("SELECT_1_TO_6"); return; }
    // Every scenario is physically placed at zero. Scan before any route movement.
    heading = desiredHeading = 0;
    x = settings::FIELD_CM - settings::START_EDGE_GAP_CM - settings::BODY_WIDTH_CM / 2;
    y = settings::FIELD_CM - settings::START_EDGE_GAP_CM - settings::BODY_LENGTH_CM / 2;
    leg = detours = 0; reverse = false; testTurn = false;
    detourResume = State::Drive;
    resetClock(in);
    beginScan(State::ScanRoute, in); reason = "SELECTING_ROUTE";
  }
  bool test90(int side, const Input& in) {
    if (active() || (side != -1 && side != 1)) return false;
    resetClock(in); testTurn = true;
    desiredHeading = (heading + side + 4) % 4;
    alignNext = State::Idle;
    beginTurn(side, in);
    return true;
  }
  bool testWheel(bool left, const Input& in) {
    if (active()) return false;
    resetClock(in); wheelLeft = left;
    enter(State::TestWheel, in);
    return true;
  }
  Output update(const Input& in) {
    if (!active()) return Output();
    const uint32_t elapsed = in.now - lastUpdate;
    if (elapsed > settings::CONTROL_GAP_MS) {
      fail("CONTROL_GAP"); return Output();
    }
    lastUpdate = in.now;
    integrate(elapsed);
    Output out;
    switch (state) {
      case State::ScanRoute:
      case State::ScanAvoid:
        if (in.scanId != scanFloor) {
          if (state == State::ScanRoute) chooseRoute(in);
          else chooseDetour(in);
        } else if (in.now - since >= settings::SCAN_TIMEOUT_MS) fail("SCAN_TIMEOUT");
        break;
      case State::Align: align(in); break;
      case State::Turn: out = turn(in); break;
      case State::Brake:
        if (in.now - since >= settings::BRAKE_MS) enter(afterBrake, in);
        break;
      case State::Drive:
      case State::ExitDrive: out = driveRoute(in); break;
      case State::OpenDrive: out = driveOpen(in); break;
      case State::ClearRear: out = clearRear(in); break;
      case State::Skirt: out = skirt(in); break;
      case State::ReturnLane: out = returnLane(in); break;
      case State::RetryClearance:
        if (in.now - pausedAt >= settings::RETRY_PAUSE_MS) retryClearance(in);
        break;
      case State::Backoff: out = backoff(in); break;
      case State::TestWheel:
        if (in.now - since >= 200) stop("WHEEL_TEST_DONE");
        else { out.left = wheelLeft ? settings::RUN_PWM : 0; out.right = wheelLeft ? 0 : settings::RUN_PWM; }
        break;
      default: break;
    }
    if (!active()) out = Output();
    previous = out;
    return out;
  }

private:
  Output previous;
  uint32_t since = 0, lastUpdate = 0, motionMs = 0;
  uint32_t scanFloor = 0, missingSince = 0, edgeSince = 0;
  uint32_t turnDurationMs = 0;
  uint32_t pausedAt = 0;
  bool missing = false, reverse = false, seenEdge = false, clearing = false;
  bool testTurn = false, wheelLeft = false;
  int turnSide = 1, desiredHeading = 0, detourHeading = 0, detourSide = 1;
  int detourPhase = 0, detours = 0;
  State afterBrake = State::Idle, alignNext = State::Drive;
  State resumeState = State::Idle, rescanState = State::ScanAvoid;
  State detourResume = State::Drive;
  float travelled = 0, clearAt = 0, outwardCm = 0;

  static int limit(int v, int bound) { return v < -bound ? -bound : (v > bound ? bound : v); }
  static bool fresh(const Range& r, const Input& in) {
    return (r.noEcho || (r.valid && r.cm >= 2 && r.cm <= 400)) &&
           in.now - r.at <= settings::RANGE_MAX_AGE_MS;
  }
  static float frontClearance(const Input& in) {
    // NO_ECHO is a separate status, not a measured 400 cm distance.
    return in.front.noEcho ? 400.0f : in.front.cm;
  }
  void fail(const char* why) { state = State::Fault; reason = why; previous = Output(); }
  void resetClock(const Input& in) {
    lastUpdate = in.now;
    previous = Output(); missing = false;
    clearanceRetries = 0;
    lastIssue = "NONE";
  }
  void pause(State waiting, const Input& in) {
    resumeState = state; pausedAt = in.now; state = waiting;
    previous = Output();
  }
  void resume(const Input& in) {
    // Preserve accumulated motor-on time and detour progress across a pause.
    since += in.now - pausedAt;
    if (clearing) edgeSince += in.now - pausedAt;
    if (missing) missingSince += in.now - pausedAt;
    state = resumeState;
  }
  void clearanceBlocked(const char* why, const Input& in) {
    lastIssue = why;
    if (clearanceRetries >= settings::MAX_CLEARANCE_RETRIES) { fail(why); return; }
    ++clearanceRetries;
    pause(State::RetryClearance, in); reason = why;
  }
  void retryClearance(const Input& in) {
    if (resumeState == State::ScanRoute || resumeState == State::ScanAvoid) {
      rescanState = resumeState;
      // First retry verifies the scan. Later retries make a short, rear-guarded retreat.
      if (clearanceRetries > 1 && !in.rearBlocked()) {
        enter(State::Backoff, in); reason = "CLEARANCE_BACKOFF";
      } else {
        beginScan(rescanState, in); reason = "CLEARANCE_RESCAN";
      }
    } else {
      // Mid-turn heading is not cardinal: keep its remaining run time and do not back up.
      resume(in); reason = "CLEARANCE_RETRY_MOTION";
    }
  }
  Output backoff(const Input& in) {
    Output out;
    if (in.rearBlocked() || travelled >= settings::RECOVERY_BACKOFF_CM) {
      scanFloor = in.scanId; brakeTo(rescanState, in); return out;
    }
    if (in.now - since >= settings::BACKOFF_TIMEOUT_MS) { fail("BACKOFF_TIMEOUT"); return out; }
    return straight(-settings::REVERSE_PWM, 0, in);
  }
  void enter(State next, const Input& in) {
    state = next; since = in.now; travelled = 0; motionMs = 0;
    seenEdge = clearing = missing = false;
  }
  void brakeTo(State next, const Input& in) {
    afterBrake = next; enter(State::Brake, in);
  }
  void beginScan(State next, const Input& in) {
    scanFloor = in.scanId; enter(next, in);
  }
  void integrate(uint32_t elapsed) {
    if (previous.left == 0 || previous.right == 0) return;
    if (state == State::Turn) { motionMs += elapsed; return; }
    if (state != State::Drive && state != State::OpenDrive && state != State::Skirt && state != State::ClearRear && state != State::Backoff &&
        state != State::ReturnLane && state != State::ExitDrive) return;
    // Linear PWM scaling is an estimate only; no movement feedback is available.
    const float average = (previous.left + previous.right) * 0.5f;
    const float cmPerSecond = average < 0 ? -average * settings::REVERSE_CM_PER_SEC / settings::REVERSE_PWM :
                                          average * settings::FORWARD_CM_PER_SEC / settings::RUN_PWM;
    float cm = cmPerSecond * elapsed / 1000.0f;
    travelled += cm;
    int direction = (heading + (previous.left < 0 ? 2 : 0)) % 4;
    if (direction == 0) y += cm;
    if (direction == 1) x += cm;
    if (direction == 2) y -= cm;
    if (direction == 3) x -= cm;
  }
  bool frontReady(const Input& in) {
    if (fresh(in.front, in)) { missing = false; return true; }
    if (!missing) { missing = true; missingSince = in.now; }
    if (in.now - missingSince >= settings::RANGE_WAIT_MS) fail("FRONT_RANGE_INVALID");
    return false;
  }
  bool motionExpired(const Input& in) {
    if (in.now - since < settings::LEG_TIMEOUT_MS) return false;
    fail("TRAVEL_TIMEOUT"); return true;
  }
  float routeScore(int direction, const Input& in) const {
    int relative = (direction - heading + 4) % 4;
    if (relative == 2) return in.rearBlocked() ? -1000.0f : 30.0f; // Blind reverse: deliberately low preference.
    const Range& r = relative == 0 ? in.front : (relative == 1 ? in.right : in.left);
    // Scan rays have different timestamps; the scan ID bounds age for side rays.
    if (!r.valid || r.cm < settings::SIDE_CLEAR_CM || r.cm > 400) return -1000;
    if ((relative == 1 && in.rightBlocked()) || (relative == 3 && in.leftBlocked())) return -1000;
    float clearance = r.cm < 150 ? r.cm : 150;
    return clearance - (relative == 0 ? 0 : 20);
  }
  void chooseRoute(const Input& in) {
    if (in.front.noEcho) { beginOpenDrive(in); return; }
    if (!in.front.valid && !in.left.valid && !in.right.valid) {
      fail("SCAN_RANGE_INVALID"); return;
    }
    float top = routeScore(3, in), bottom = routeScore(2, in);
    if (top < 0 && bottom < 0) { clearanceBlocked("NO_START_CLEARANCE", in); return; }
    route = top >= bottom ? Route::Top : Route::Bottom;
    reason = route == Route::Top ? "TOP_ROUTE" : "BOTTOM_ROUTE";
    planLeg(in);
  }
  int routeDirection() const {
    if (route == Route::Top) return leg == 0 ? 3 : 2;
    return leg == 0 ? 2 : (leg == 1 ? 3 : 2);
  }
  bool finalLeg() const { return leg == (route == Route::Top ? 1 : 2); }
  float distanceToBoundary() const {
    return routeDirection() == 3 ? x : y;
  }
  void planLeg(const Input& in) {
    int direction = routeDirection();
    reverse = leg == 0 && (direction - heading + 4) % 4 == 2;
    desiredHeading = reverse ? heading : direction;
    alignNext = State::Drive;
    brakeTo(State::Align, in);
  }
  void alignTo(int direction, State next, const Input& in) {
    reverse = false; desiredHeading = direction; alignNext = next;
    brakeTo(State::Align, in);
  }
  void align(const Input& in) {
    int delta = (desiredHeading - heading + 4) % 4;
    if (!delta) {
      if (alignNext == State::ScanRoute || alignNext == State::ScanAvoid) beginScan(alignNext, in);
      else enter(alignNext, in);
      return;
    }
    // A half-turn is deliberately executed as two separate 90-degree turns.
    beginTurn(delta == 3 ? -1 : 1, in);
  }
  void beginTurn(int side, const Input& in) {
    turnSide = side;
    turnDurationMs = side < 0 ? settings::TURN_LEFT_90_MS : settings::TURN_RIGHT_90_MS;
    enter(State::Turn, in);
  }
  Output turn(const Input& in) {
    Output out;
    if (in.now - since >= settings::TURN_TIMEOUT_MS) { fail("TURN_TIMEOUT"); return out; }
    if (!frontReady(in)) return out;
    if (frontClearance(in) <= settings::TURN_STOP_CM || in.anyIrBlocked()) {
      clearanceBlocked("TURN_CLEARANCE", in); return out;
    }
    if (motionMs >= turnDurationMs) {
      heading = (heading + turnSide + 4) % 4;
      if (testTurn) { testTurn = false; brakeTo(State::Idle, in); reason = "TURN_TEST_DONE"; }
      else brakeTo(State::Align, in);
      return out;
    }
    out.left = -turnSide * settings::TURN_PWM;
    out.right = turnSide * settings::TURN_PWM;
    return out;
  }
  Output straight(int speed, int followSide, const Input& in) const {
    (void)followSide;
    Output out;
    if (speed <= 0) { out.left = speed; out.right = speed; return out; }
    if (in.irFrontLeft && !in.irFrontRight) {
      // Obstacle on left: swerve right by slowing down right wheel
      out.left = speed;
      out.right = limit(speed - settings::SIDE_CORRECTION_PWM, settings::MAX_PWM);
      if (out.right < 0) out.right = 0;
      return out;
    }
    if (in.irFrontRight && !in.irFrontLeft) {
      // Obstacle on right: swerve left by slowing down left wheel
      out.left = limit(speed - settings::SIDE_CORRECTION_PWM, settings::MAX_PWM);
      if (out.left < 0) out.left = 0;
      out.right = speed;
      return out;
    }
    out.left = speed;
    out.right = speed;
    return out;
  }
  void nextLeg(const Input& in) {
    // Route corners reset the coarse timed position estimate to the planned lane.
    if (routeDirection() == 3) x = settings::LANE_CENTER_CM;
    else y = settings::LANE_CENTER_CM;
    ++leg; planLeg(in);
  }
  void avoid(const Input& in) {
    if (++detours > settings::MAX_DETOURS) { fail("TOO_MANY_DETOURS"); return; }
    detourResume = state == State::OpenDrive ? State::OpenDrive : State::Drive;
    detourHeading = state == State::OpenDrive ? heading : routeDirection();
    // A rear obstacle must be faced before using front/side scanning.
    if (reverse) {
      alignTo(detourHeading, State::ScanAvoid, in);
      scanFloor = in.scanId;
    } else { beginScan(State::ScanAvoid, in); }
    reason = "OBSTACLE";
  }
  Output driveRoute(const Input& in) {
    Output out;
    if (fresh(in.front, in) && in.front.noEcho) {
      if (reverse) {
        reverse = false; brakeTo(State::OpenDrive, in); reason = "NO_ECHO_FORWARD"; return out;
      }
      beginOpenDrive(in); return driveOpen(in);
    }
    if (state != State::ExitDrive && motionExpired(in)) return out;
    if (!finalLeg() && distanceToBoundary() <= settings::LANE_CENTER_CM) {
      nextLeg(in); return out;
    }
    if (reverse) {
      if (in.rearBlocked()) {
        if (distanceToBoundary() <= settings::WALL_ZONE_CM) nextLeg(in);
        else avoid(in);
        if (state != State::Fault) brakeTo(State::ClearRear, in);
        return out;
      }
      return straight(-settings::REVERSE_PWM, 0, in);
    }
    if (!frontReady(in)) return out;
    if (in.front.cm <= settings::FRONT_STOP_CM) {
      if (!finalLeg() && distanceToBoundary() <= settings::WALL_ZONE_CM) nextLeg(in);
      else avoid(in);
      return out;
    }
    if (in.irFrontLeft && in.irFrontRight) { clearanceBlocked("BOTH_SIDES_BLOCKED", in); return out; }
    if (finalLeg() && y < -settings::BODY_LENGTH_CM / 2 && state != State::ExitDrive) {
      enter(State::ExitDrive, in); reason = "EXIT_CONTINUE";
    }
    int speed = in.front.cm < settings::FRONT_SLOW_CM ? settings::SLOW_PWM : settings::RUN_PWM;
    return straight(speed, in.irFrontLeft ? -1 : (in.irFrontRight ? 1 : 0), in);
  }
  void beginOpenDrive(const Input& in) {
    reverse = false; enter(State::OpenDrive, in); reason = "NO_ECHO_FORWARD";
  }
  Output driveOpen(const Input& in) {
    Output out;
    if (!frontReady(in)) return out;
    if (frontClearance(in) <= settings::FRONT_STOP_CM) { avoid(in); return out; }
    if (in.irFrontLeft && in.irFrontRight) { clearanceBlocked("BOTH_SIDES_BLOCKED", in); return out; }
    const int speed = frontClearance(in) < settings::FRONT_SLOW_CM ? settings::SLOW_PWM : settings::RUN_PWM;
    // No estimated-distance corner or leg deadline: run until detection or user stop.
    return straight(speed, in.irFrontLeft ? -1 : (in.irFrontRight ? 1 : 0), in);
  }
  Output clearRear(const Input& in) {
    Output out;
    if (motionExpired(in)) return out;
    if (!in.rearBlocked() && travelled >= settings::REAR_RELEASE_CM) {
      brakeTo(State::Align, in); return out;
    }
    if (travelled >= settings::REAR_RELEASE_MAX_CM) { fail("REAR_IR_STUCK"); return out; }
    if (!frontReady(in)) return out;
    if (frontClearance(in) <= settings::FRONT_STOP_CM || (in.irFrontLeft && in.irFrontRight)) {
      clearanceBlocked("REAR_RELEASE_BLOCKED", in); return out;
    }
    return straight(settings::REVERSE_PWM, 0, in);
  }
  bool roomInside(int direction) const {
    // Pose is only an estimate; this excludes detours toward an immediately adjacent wall.
    if (y < 0 || detourResume == State::OpenDrive) return true;
    float available = direction == 0 ? settings::FIELD_CM - y :
                      (direction == 1 ? settings::FIELD_CM - x : (direction == 2 ? y : x));
    return available > settings::BODY_CLEAR_CM + settings::BODY_LENGTH_CM / 2;
  }
  void chooseDetour(const Input& in) {
    if (in.front.noEcho) { beginOpenDrive(in); return; }
    if (!in.front.valid && !in.left.valid && !in.right.valid) { fail("SCAN_RANGE_INVALID"); return; }
    bool l = in.left.valid && in.left.cm >= settings::SIDE_CLEAR_CM && !in.leftBlocked() && roomInside((heading + 3) % 4);
    bool r = in.right.valid && in.right.cm >= settings::SIDE_CLEAR_CM && !in.rightBlocked() && roomInside((heading + 1) % 4);
    if (!l && !r) { clearanceBlocked("NO_DETOUR_CLEARANCE", in); return; }
    detourSide = l && (!r || in.left.cm >= in.right.cm) ? -1 : 1;
    detourPhase = 0;
    alignTo((detourHeading + detourSide + 4) % 4, State::Skirt, in);
  }
  Output skirt(const Input& in) {
    Output out;
    if (motionExpired(in)) return out;
    if (travelled > settings::MAX_SKIRT_CM) { fail("EDGE_NOT_FOUND"); return out; }
    if (!frontReady(in)) return out;
    if (frontClearance(in) <= settings::FRONT_STOP_CM) { clearanceBlocked("DETOUR_BLOCKED", in); return out; }
    int follow = -detourSide;
    bool edge = follow < 0 ? in.leftBlocked() : in.rightBlocked();
    bool opposite = follow < 0 ? in.rightBlocked() : in.leftBlocked();
    if (opposite) { clearanceBlocked("DETOUR_SIDE_BLOCKED", in); return out; }
    if (edge) { seenEdge = true; clearing = false; }
    else if (seenEdge && !clearing) { clearing = true; edgeSince = in.now; clearAt = travelled; }
    if (clearing && in.now - edgeSince >= settings::EDGE_CONFIRM_MS &&
        travelled - clearAt >= settings::BODY_CLEAR_CM && frontClearance(in) >= settings::SIDE_CLEAR_CM) {
      if (detourPhase == 0) {
        outwardCm = travelled; detourPhase = 1;
        alignTo(detourHeading, State::Skirt, in);
      } else {
        alignTo((detourHeading - detourSide + 4) % 4, State::ReturnLane, in);
      }
      return out;
    }
    return straight(settings::SLOW_PWM, follow, in);
  }
  Output returnLane(const Input& in) {
    Output out;
    if (motionExpired(in)) return out;
    if (travelled >= outwardCm) {
      alignTo(detourHeading, detourResume, in); reason = "RESUME_ROUTE"; return out;
    }
    if (!frontReady(in)) return out;
    if (frontClearance(in) <= settings::FRONT_STOP_CM || (in.irFrontLeft && in.irFrontRight)) {
      clearanceBlocked("RETURN_LANE_BLOCKED", in); return out;
    }
    return straight(settings::SLOW_PWM, 0, in);
  }
};

inline const char* stateName(State s) {
  switch (s) {
    case State::Idle: return "IDLE";
    case State::ScanRoute: return "SCAN_ROUTE";
    case State::Align: return "ALIGN";
    case State::Turn: return "TURN_90";
    case State::Brake: return "BRAKE";
    case State::Drive: return "DRIVE";
    case State::OpenDrive: return "OPEN_FORWARD";
    case State::ClearRear: return "CLEAR_REAR";
    case State::ScanAvoid: return "SCAN_AVOID";
    case State::Skirt: return "FOLLOW_EDGE";
    case State::ReturnLane: return "RETURN_LANE";
    case State::ExitDrive: return "EXIT_CONTINUE";
    case State::TestWheel: return "WHEEL_TEST";
    case State::RetryClearance: return "RETRY_CLEARANCE";
    case State::Backoff: return "BACKOFF";
    case State::Fault: return "FAULT";
  }
  return "UNKNOWN";
}
}
