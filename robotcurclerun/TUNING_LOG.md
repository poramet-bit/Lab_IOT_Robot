# robotcurclerun.ino — Debug & Tuning Log

Summary of the debugging/tuning session for the LM393 encoder auto-balance
drive in `robotcurclerun.ino`.

## Bugs fixed

- `right_turnback()`: trim was applied as `speed_motorR - B_R` on the ENB
  (right motor) write. Every other function pairs ENA with `B_R` and ENB
  with `B_L`; this one used `B_R` for both. Fixed to `speed_motorR - B_L`.
- `#include` for the animation header used an absolute, machine-specific
  path (`/home/poramet/bass_github/...`). Fixed to a relative include
  (`../animation/animation.h`) so the sketch is portable.

## Removed

- Servo motor code entirely: `Servo.h` include, `Servo servo` object, `SV`
  pin, `servo.attach()` in `setup()`, and the `Sv()` helper function. Not
  used by the drive logic; the calls to `Sv()` in `loop()` were already
  commented out.

## Encoder-based speed balancing

`forward()` / `backward()` already implement closed-loop balancing using
the LM393 wheel encoders (`pulse_count_L`, `pulse_count_R`):

- Each animation frame (~66 ms), the code reads the pulse delta since the
  start of the current `forward()`/`backward()` call for both wheels.
- `error = delta_L - delta_R`. Outside a 1-pulse deadband, PWM is adjusted
  by `Kp * error`, clamped to ±35.
- A static hardware trim, `MOTOR_L_RATIO`, is applied to the left motor's
  base PWM to compensate for the left motor naturally spinning faster than
  the right at the same PWM value.

## Tuning trail (MOTOR_L_RATIO)

| Value | Kp  | Deadband | Result (from `printEncoderStatus()` logs) |
|-------|-----|----------|---------------------------------------------|
| 0.850 | 1.4 | >1 pulse | Left consistently ahead of right; drift plateaus around +60–65 pulses cumulative. Robot veers right. |
| 0.83  | 1.4 | >1 pulse | Left still ahead, diff grows steadily (~+3/call), not converging. |
| 0.81  | 1.4 | >1 pulse | (not logged before moving on) |
| 0.79  | 1.4 | >1 pulse | One log run showed Right stuck at 0 while Left froze — diagnosed as a **hardware** issue (wheel/encoder not physically turning), not a regression from this value. |
| 0.82  | 1.4 | >1 pulse | RPM nearly matched (~1.8% diff, 25.65 vs 25.15 rev) — but robot still visibly veered right. Since RPM was balanced, this pointed to a **mechanical** cause (wheel diameter/tire wear, chassis alignment, uneven weight/friction) rather than a PWM/software issue. |
| 0.825 | 1.4 | >1 pulse | After the mechanical check, nudged up half a step from 0.82 because 0.82-ish settings had started drifting left instead of right (overshoot from 0.02 steps being too coarse). Reported as "starting to go straight." |
| 0.82  | 1.4 | >1 pulse | Path-routine hardware run, left-drift starting ~80cm into straight. Nudged down. |
| 0.815 | 1.4 | >1 pulse | Flipped: visibly veered right. True balance point bracketed between 0.815 and 0.82. |
| 0.818 | 1.4 | >1 pulse | Fresh path-routine log: Right pulses pulled ahead of Left, gap widening across segments 0-1 (e.g. 1219 vs 1281, diff 62). Visually still veered left. |
| 0.816 | 1.4 | >1 pulse | Bisecting between 0.815 (right) and 0.818 (left). **Current value in the sketch — not yet confirmed on hardware.** |

## Open items / follow-ups

- If it drifts right again: nudge `MOTOR_L_RATIO` up slightly (e.g. 0.83).
  If it drifts left: nudge down slightly (e.g. 0.82).
- Mechanical causes worth checking if PWM tuning alone won't hold a
  straight line: wheel/tire diameter match, chassis/wheel alignment,
  weight distribution, gearbox friction difference between sides.
- Dead code noted but not removed: global `speed_motorL`, `speed_motorR`,
  `B_L`, `B_R` variables are shadowed by same-named parameters in
  `forward()`/`backward()` (and other motion functions), so calling these
  functions with no arguments always uses their hardcoded defaults and
  ignores the globals entirely.

## left_turn() direction bug

`left_turn()` drove the left wheel forward with the right wheel stationary
(default `speed_motorR = 0`). For a single-wheel pivot, that swings the
chassis toward the *stationary* side — i.e. it physically turned the robot
right, not left. Fixed by driving the right wheel forward with the left
stationary instead (swapped `IN1`/`IN2` for `IN3`/`IN4`, swapped the default
speed params). `right_turn()` was left untouched — not reported as broken.

## Path routine (fixed 5-segment route) + point-turn spins

Added a fixed route driven by a `PathStep[] PATH` table: drive straight for
`distance_m`, spin 360° in place (a flourish), optionally follow with a 90°
turn, repeat for 5 segments, then stop. Implemented via `spinInPlace()`
(encoder-gated in-place rotation) and `runPathStepManeuver()`.

### Distance calibration (field-measured)

Procedure: `resetEncoders()` implicitly via power-on (never called mid-run),
mark start/end points on the floor, run `forward()` to the end mark, read
`printEncoderStatus()` at that point, average both wheels' pulses, divide by
measured distance.

| Measured distance | Avg pulses | Result |
|---|---|---|
| 450 cm (4.5 m) | ~473 (L=469, R=477 at trigger) | `PULSES_PER_METER = 105.1` |

Theoretical estimate from wheel geometry (65 mm wheel diameter, 20 slots/rev)
was 97.9 pulses/m — off by ~7%, attributed to wheel slip / effective rolling
diameter differing from nominal.

### Spin direction

`spinInPlace(degrees, clockwise)`'s CW/CCW branches were derived
geometrically (left-forward + right-backward = clockwise) but came out
backwards on the physical robot. Branches were swapped to match observed
behavior; `PATH[]`'s per-step direction flags (`spin_clockwise`,
`turn_clockwise`) did not need to change.

### Spin angle calibration (point-turn scrub)

Point turns scrub the tires sideways instead of rolling cleanly, so pulses
predicted from wheelbase geometry alone under-rotate the robot. Empirical
trail, target = commanded 90°:

| `SPIN_SLIP_FACTOR` | Real rotation observed |
|---|---|
| 1.0 (uncorrected) | ~45° |
| 2.0 | ~105° (overshoot) |
| 1.71 (`2.0 * 90/105`) | **current value** |

Still coarse: `DISK_SLOTS = 20` gives ~10.9°/pulse resolution before the slip
factor, so exact-degree accuracy has a real floor. Scrub also isn't constant
— depends on surface and weight — so this factor may need re-tuning if the
robot moves to a different floor.

### spinInPlace() missing left-motor trim

`spinInPlace()` initially drove both wheels at equal PWM (`spinSpeed`),
unlike `forward()`/`backward()` which apply `MOTOR_L_RATIO` to the left
motor. Reported symptom: left wheel spinning noticeably harder during
maneuvers, straight-line driving looking different from before the path
routine was added. Fixed by scaling `ENA`'s PWM by `MOTOR_L_RATIO` in
`spinInPlace()` too, same as the straight-drive functions.

## Open items / follow-ups (path routine)

- If post-spin straight-line drift persists after the `MOTOR_L_RATIO` fix
  above, check battery voltage — spins draw much more current than straight
  driving (motors fighting tire scrub), and voltage sag can desync the two
  motors' response even with the same PWM ratio applied.
- `PULSES_PER_DEGREE`/`SPIN_SLIP_FACTOR` are tuned from rough visual angle
  estimates, not a protractor measurement — revisit with a marked-angle
  floor test if step-to-step heading error compounds visibly over the
  5-segment route.

## Single-wheel pivot (spin) rolled out to legs 1-4 + turn_left() direction bug

### Single-wheel pivot applied to all legs

Leg 1 already used `pivotLeftForward()`/`pivotLeftBackward()` (single-wheel
pivot: one wheel stationary, the other drives) for its spin. Legs 2-4 were
still using `turn_left()`/`turn_right()` (two-wheel pivot) for their spin
step. Switched legs 2-4's spin call to the single-wheel pivot too, matching
direction to the original comment (`spin left` -> `pivotLeftBackward`,
`spin right` -> `pivotLeftForward`).

### turn_left() direction bug

`turn_left()`'s `digitalWrite(IN1..IN4)` block was byte-for-byte identical to
`turn_right()`'s (both wrote left-forward/right-backward), despite the
comment above it saying "Left backward, Right forward". Result: calling
`turn_left()` always spun the robot right, regardless of which function was
called. Fixed both the main spin direction and the counter-torque brake
block inside `turn_left()` to actually drive left-backward/right-forward.

### Spin vs. turn value mix-up in loop()

While re-tuning, `LEG{n}_SPIN_DEG` and `LEG{n}_TURN_DEG` constants ended up
holding each other's values (small ~0-90° corner-turn numbers sitting in
`SPIN_DEG`, large ~360-630° full-spin numbers sitting in `TURN_DEG`). Rather
than re-guess the intended numbers, swapped which constant feeds which call
in `loop()`: the pivot spin call now takes `LEG{n}_TURN_DEG` and `turn_right`/
`turn_left` now takes `LEG{n}_SPIN_DEG`, so the large values drive the full
spin and the small values drive the corner turn again.

### Missing encoder feedback during turns

Audited whether any turn/spin function uses the encoders to correct L/R
balance mid-turn (not just as a stop condition) — none did:

- `turn_right()`/`turn_left()`: encoder pulses were only checked as an
  average (`(pulse_count_L + pulse_count_R) / 2 >= target_pulses`) to decide
  when to stop; PWM stayed frozen at launch value for the whole turn.
- `pivotLeftForward()`/`pivotLeftBackward()`: didn't use the encoder at all
  to stop — pure `millis()`-based timing; `pulse_count_L` was only read
  *after* stopping, as a "did the wheel even turn" diagnostic.

This explains inconsistent real-world rotation angle from run to run (no
correction for wheel slip, friction differences, or battery sag mid-turn).
Fixed both:

- `pivotLeftForward()`/`pivotLeftBackward()`: now stop on `pulse_count_L`
  reaching a target computed from the single-wheel pivot's arc (radius =
  full `TRACK_WIDTH_CM`, so 2x the two-wheel pivot's arc at the same angle),
  with a timeout fallback instead of pure timing.
- `turn_right()`/`turn_left()`: added a live correction loop mid-turn using
  `error = pulse_count_L - pulse_count_R` through the existing `Kp_enc`/
  `Ki_enc` gains (same pattern as `forward()`), biasing per-wheel PWM so
  both sides rotate the same amount instead of running both wheels at a
  fixed speed for the whole turn.
