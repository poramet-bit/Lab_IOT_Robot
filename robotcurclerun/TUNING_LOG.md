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
| 0.825 | 1.4 | >1 pulse | After the mechanical check, nudged up half a step from 0.82 because 0.82-ish settings had started drifting left instead of right (overshoot from 0.02 steps being too coarse). Reported as "starting to go straight." **Current value in the sketch.** |

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
