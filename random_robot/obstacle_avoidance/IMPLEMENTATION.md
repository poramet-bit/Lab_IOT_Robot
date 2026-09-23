# Obstacle robot: timed motion

Current scope: UNO R4 WiFi obstacle/exit sketch with four corner IR sensors and
the same zero-degree placement for all six scenarios. No preliminary dice rotation
or wheel feedback. Earlier encoder-based implementation and retries are superseded.

- Settings.h: motor wiring/trim, sensor thresholds, separate left/right 90-degree durations,
  estimated forward/reverse speeds and bounded clearance retries.
- Navigation.h: physical startup from zero followed immediately by a braked route scan,
  top/bottom route selection, edge following and guarded recovery. Accumulate only motor-on
  time; preserve it across clearance pauses. Timed x/y estimates are not measured position.
  Scenario numbers remain selection/status identifiers; equal initial headings and sensor
  inputs now produce the same route behavior for all six numbers.
- obstacle_avoidance.ino: BLE/USB commands, servo/sonar/IR sampling, motor outputs and timed
  telemetry. No wheel input pins, wheel interrupts, pulse counters, pulse balancing or stall state.
- Four corner IR inputs: front-left A3, front-right A4, rear-left A5, rear-right D2.
  Telemetry reports IR_FL/FR/BL/BR. Either rear detection guards reverse/backoff;
  all corners guard turning. Route scans and edge following combine front/rear on each side,
  while forward steering uses only the front pair so the robot can release a rear obstacle.
- D3/D11 are unused. User stop, BLE disconnect, missing-range checks and phase timeouts remain.

Defaults needing physical calibration: TURN_LEFT_90_MS=390, TURN_RIGHT_90_MS=390 (local values retained),
FORWARD_CM_PER_SEC=30*255/140 and REVERSE_CM_PER_SEC=15*255/85.
All movement PWM values, including raised-wheel diagnostics, are now 255 before RIGHT_TRIM.
Travel estimates were linearly extrapolated from the previous PWM settings; retune both
travel rates and turning durations on the actual robot at full power.
The turn durations are initial tuning values, not measurements from this robot.

Verification: host controller and sketch tests check both routes, all startup scenarios,
timed turns without wheel data, interrupted turns, sensor waits, rollover, recovery limits,
both rear detections, edge clearance at both ends, four-corner turn guards and pin telemetry,
stop/disconnect and unused former input pins. Compile target:
arduino:renesas_uno:unor4wifi with ArduinoBLE and Servo. No hardware upload or field test.
