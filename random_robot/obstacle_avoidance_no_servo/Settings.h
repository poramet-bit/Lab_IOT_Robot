#pragma once
#include <stdint.h>

namespace settings {
// Calibration from Max_Speed/obstacle_avoidance_auto/obstacle_avoidance_auto.ino.
// These pin names identify driver channels; drive() maps the logical wheel sides.
constexpr int LEFT_PWM = 5, LEFT_A = 6, LEFT_B = 7;
constexpr int RIGHT_PWM = 10, RIGHT_A = 9, RIGHT_B = 8;
constexpr float RIGHT_TRIM = 46800.0f / 55500.0f;
constexpr bool MOTOR_SIDES_SWAPPED = true;
constexpr bool LEFT_INVERTED = false, RIGHT_INVERTED = false;
constexpr bool IR_SIDES_SWAPPED = true;
constexpr bool IR_ACTIVE_LOW = true;
constexpr int RUN_PWM = 180, TURN_PWM = 160, REVERSE_PWM = 130;
constexpr int BRAKE_PWM = 255;
constexpr bool SERVO_MIRRORED = false;
constexpr int SERVO_FRONT = 90, SERVO_SIDE_ANGLE = 60;
constexpr int SERVO_LEFT = SERVO_MIRRORED ? SERVO_FRONT - SERVO_SIDE_ANGLE
                                       : SERVO_FRONT + SERVO_SIDE_ANGLE;
constexpr int SERVO_RIGHT = SERVO_MIRRORED ? SERVO_FRONT + SERVO_SIDE_ANGLE
                                        : SERVO_FRONT - SERVO_SIDE_ANGLE;
constexpr uint32_t SERVO_SETTLE_MS = 300;
constexpr int SONAR_SAMPLES = 3;
constexpr float SIDE_CLEAR_CM = 25.0f;
constexpr uint32_t TURN_LEFT_90_MS = 450, TURN_RIGHT_90_MS = 450;
// Keep the existing timed retreat: this sketch does not read wheel encoders.
constexpr uint32_t START_DELAY_MS = 3000, BRAKE_MS = 250, BACKUP_MS = 180;
// Driving threshold only; route selection compares the full measured sonar range.
constexpr float FRONT_STOP_CM = 25.0f, TURN_STOP_CM = 8.0f;
constexpr uint32_t CONTROL_MS = 10, CONTROL_GAP_MS = 250;
constexpr uint32_t PING_INTERVAL_MS = 60, ECHO_TIMEOUT_US = 30000;
constexpr uint32_t RANGE_MAX_AGE_MS = 300, RANGE_WAIT_MS = 1500;
constexpr uint32_t TURN_TIMEOUT_MS = 3000;
constexpr int NO_ECHO_SAMPLES = 3;
// Shared budget for turns/backups without a sustained forward run.
constexpr int MAX_AVOID_ACTIONS = 6;
constexpr uint32_t FORWARD_PROGRESS_MS = 500;
static_assert(RUN_PWM > 0 && RUN_PWM <= 255 && TURN_PWM > 0 && TURN_PWM <= 255 &&
              REVERSE_PWM > 0 && REVERSE_PWM <= 255, "Use 8-bit motor PWM");
static_assert(TURN_LEFT_90_MS > 0 && TURN_LEFT_90_MS < TURN_TIMEOUT_MS &&
              TURN_RIGHT_90_MS > 0 && TURN_RIGHT_90_MS < TURN_TIMEOUT_MS, "Check turn timing");
}
