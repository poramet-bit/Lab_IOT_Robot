#pragma once
#include <stdint.h>

namespace settings {
// Motor wiring/direction and trim from IOT-Robot/Robot/robotline_fullspeed.
constexpr int LEFT_PWM = 5, LEFT_A = 6, LEFT_B = 7;
constexpr int RIGHT_PWM = 10, RIGHT_A = 9, RIGHT_B = 8;
constexpr float RIGHT_TRIM = 46800.0f / 55500.0f;
constexpr bool IR_ACTIVE_LOW = true; // Verify OUT with v; invert if necessary.
constexpr int SERVO_LEFT = 180, SERVO_FRONT = 90, SERVO_RIGHT = 0;
constexpr uint32_t SERVO_SETTLE_MS = 300, PING_INTERVAL_MS = 65;
constexpr uint32_t ECHO_TIMEOUT_US = 26000, RANGE_MAX_AGE_MS = 300;
constexpr int SONAR_SAMPLES = 3; // Confirm open front only after this many consecutive timeouts.
constexpr uint32_t RANGE_WAIT_MS = 1500, CONTROL_MS = 10, CONTROL_GAP_MS = 250;
constexpr uint32_t SCAN_TIMEOUT_MS = 5000, TURN_TIMEOUT_MS = 4000;
constexpr uint32_t LEG_TIMEOUT_MS = 30000, BRAKE_MS = 150;
// Retry budgets apply to one f / l / r run; s always cancels pending retries.
constexpr int MAX_CLEARANCE_RETRIES = 3;
constexpr uint32_t RETRY_PAUSE_MS = 500, BACKOFF_TIMEOUT_MS = 1500;
constexpr float RECOVERY_BACKOFF_CM = 8.0f;
// Full PWM in every movement mode, as requested. RIGHT_TRIM follows robotline_fullspeed.
constexpr int RUN_PWM = 255, SLOW_PWM = 255, REVERSE_PWM = 255, TURN_PWM = 255;
constexpr int MAX_PWM = 255;
constexpr int SIDE_CORRECTION_PWM = 10;
// Existing local timings; retune using l/r after changing motor power.
// Count only time the motors are commanded to turn; exclude stops and scan waits.
constexpr uint32_t TURN_LEFT_90_MS = 390, TURN_RIGHT_90_MS = 390;
// Extrapolate the previous estimates (30 cm/s at PWM 140, 15 cm/s at PWM 85).
// Linear scaling is only a starting estimate, not a measurement at full PWM.
constexpr float FORWARD_CM_PER_SEC = 30.0f * RUN_PWM / 140.0f;
constexpr float REVERSE_CM_PER_SEC = 15.0f * REVERSE_PWM / 85.0f;
constexpr float FIELD_CM = 300.0f, BODY_WIDTH_CM = 17.0f, BODY_LENGTH_CM = 30.0f;
constexpr float START_EDGE_GAP_CM = 40.0f;
constexpr float LANE_CENTER_CM = 35.0f; // Exit toward the left half, with turning clearance.
constexpr float FRONT_STOP_CM = 20.0f, FRONT_SLOW_CM = 50.0f, TURN_STOP_CM = 8.0f;
constexpr float SIDE_CLEAR_CM = 35.0f, WALL_ZONE_CM = 65.0f;
constexpr float BODY_CLEAR_CM = BODY_LENGTH_CM + 5.0f;
constexpr float REAR_RELEASE_CM = 12.0f, REAR_RELEASE_MAX_CM = 40.0f;
constexpr float MAX_SKIRT_CM = 140.0f;
constexpr uint32_t EDGE_CONFIRM_MS = 150;
constexpr int MAX_DETOURS = 8;
static_assert(RUN_PWM <= MAX_PWM && SLOW_PWM <= MAX_PWM && REVERSE_PWM <= MAX_PWM &&
              TURN_PWM <= MAX_PWM && MAX_PWM <= 255, "PWM is 8-bit");
static_assert(BODY_WIDTH_CM < 100 && LANE_CENTER_CM > BODY_LENGTH_CM / 2, "Check body clearance");
static_assert(TURN_LEFT_90_MS > 0 && TURN_RIGHT_90_MS > 0 &&
              TURN_LEFT_90_MS < TURN_TIMEOUT_MS && TURN_RIGHT_90_MS < TURN_TIMEOUT_MS, "Check turn duration");
static_assert(FORWARD_CM_PER_SEC > 0 && REVERSE_CM_PER_SEC > 0 && RIGHT_TRIM > 0 &&
              RUN_PWM > 0 && REVERSE_PWM > 0, "Check timed travel calibration");
}
