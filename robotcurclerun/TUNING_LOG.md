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
