# Five-sensor line-following robot

Status: implemented in `line_following/line_following.ino` and `line_controller.h`; physical acceptance remains to be tested. User requested implementation after the specification interview.

## Hardware and scope

Separate sketch; preserve route/test sketches. UNO R4 WiFi, ArduinoBLE, five TCRT5000 analog outputs. Physical sides are viewed from the rear toward the front.

- Left-to-right sensor pins: A5, A4, A3, A2, A1. A0 unused; no I2C gyro or LED animation in this sketch.
- Physical left motor: PWM=10, IN=8/9 (legacy R). Physical right: PWM=5, IN=7/6 (legacy L).
- Black line 2–2.5 cm wide on bright yellow-brown wood-colored floor. One main route, decoy lines with straight continuation, plus 90-degree turns.
- Rear support wheel fixed pointing straight; vehicle approximately 25 cm long, sensors at most 10 cm ahead of front wheels.
- User selected sensor-face height 2.5 mm. Exact 5CH board spacing is unknown; use ordered positions [-2,-1,0,1,2], not invented dimensions.

## Trigger and stopping

BLE name IOT-LINE with existing Nordic UART UUIDs. Boot braked. f/START begins following, s/STOP brakes, v returns one sensor report while stopped. No mandatory calibration and no c command. BLE disconnect brakes; reconnection does not resume motion. Repeated f during motion cannot extend a search timeout.

No finish marker is defined. End-of-line enters lost-line recovery. Without prior side evidence brake immediately; otherwise search for at most 2000 ms, then brake and wait for f.

## Controller

Read 10-bit ADC every 10 ms. Configurable per-channel thresholds initially 512, hysteresis 20, black-high polarity initially true. These are unvalidated hardware defaults; use v over black/floor to verify and tune manually. No persistent calibration storage.

Use binary detections and the mean of ordered sensor positions. If the center sees the line, restrict steering to the contiguous sensor group containing the center, excluding disconnected outer decoys. A symmetric/all-black pattern steers straight. This implements straight preference from current readings, not guaranteed recognition of every decoy geometry.

Normal steering: physical left PWM = base + correction; physical right = base - correction, correction = gain times mean position. Defaults base=35000, gain=7000, maximum=50000. Clamp negative output to zero (dynamic brake).

Remember the last unambiguous off-center side. Outer-only or outer+inner detections on one side without center/opposite detections initiate a corner search. No detections initiate recovery toward the remembered side. Inner wheel brakes and outer wheel drives forward at search PWM 35000. Preserve search direction and start time throughout recovery. Three consecutive center detections reacquire the line. A 2000 ms deadline brakes even if still detecting an outer sensor. Timers handle unsigned wraparound.

## Reporting and acceptance

Status on state transitions, not continuous telemetry. v reports raw values and five black flags in physical order. BLE reports use paced 20-byte chunks without blocking motor control.

Host tests verify stopped outputs, left/right correction, center preference over separated decoys, left/right corner entry, remembered search side, timeout, stable reacquisition, stop latching, and timer wraparound. A mock Arduino/BLE integration check verifies physical pin outputs, immediate stop, repeated-start timeout preservation, and disconnect braking.

Hardware acceptance: v must distinguish black/floor for each sensor; verify motor direction with wheels raised; test straight, decoy continuation, both 90-degree corners, lost-line recovery, timeout, s and BLE disconnect. Tune speed/gain against fixed-caster behavior. Host checks do not certify physical path accuracy or an Arduino board build.

## Optical evidence

[Vishay TCRT5000 datasheet](https://www.vishay.com/docs/83760/tcrt5000.pdf) gives peak response at 2.5 mm and a 0.2–15 mm range above 20% relative collector current. These component characteristics do not guarantee contrast on the actual floor. The user's initial 25 mm height was superseded by 2.5 mm.
