# Robot workspace notes

## Line-following workflow — interview in progress (2026-09-09)

- Requested: a separate sketch for line following with five sensors.
- Confirmed sensor order, physical left to right: A5, A4, A3, A2, A1. Board A0 is unused.
- Sensor model: TCRT5000; follow a black line on a bright yellow-brown wood-colored background.
- Confirmed Arduino UNO R4 WiFi and AO analog outputs on all five sensors.
- User selected 2.5 mm mounting height, superseding 25 mm. Board known only as IR line tracking sensor / TCRT5000 5CH; exact sensor spacing unknown. Use ordered positions for controller design.
- Vehicle length is approximately 25 cm front to rear; this is not a measured wheelbase.
- Rear support wheel is fixed pointing straight. Sensor array is ahead of the front wheels by no more than 10 cm.
- Define physical left/right while looking from the rear toward the front of the robot.
- Last user-confirmed mapping: code L is physical right; code R is physical left.
- Existing raw sketch: code L uses ENA=5, IN1=7, IN2=6; code R uses ENB=10, IN3=8, IN4=9.
- Existing encoder mapping: code L=3, code R=11.
- The existing gyro code uses A4/A5 for I2C; decide whether the new sketch needs gyro before assigning line sensors.
- Implemented defaults: no prior side detection means brake; end of line follows the 2-second recovery rule; v reads sensors while stopped. Exact spacing is not needed for ordered sensor positions.
- Do not assume raw route speed calibration transfers unchanged to line following.
- User invoked loop-me for specification, then explicitly requested implementation. Sketch: line_following/line_following.ino. Specification: workflows/line-following.md.
- Track line width confirmed as 2–2.5 cm with 90-degree corners. User corrected the junction description: one main route, decoy lines with a forward continuation, no true branch-choice requirement.
- User accepted BLE f=start following and s=brake/stop.
- Confirmed lost-line search uses one or two detections on the same side. Prefer straight at junctions if a forward continuation exists. Search timeout is 2 seconds, then brake and wait for f.
- User deferred calibration: no c command or mandatory calibration procedure in this version.
