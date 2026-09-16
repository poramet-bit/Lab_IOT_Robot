# robotcurclerun.ino — Session Summary (2026-09-01)

Condensed record of the debugging/tuning session for `robotcurclerun.ino`,
carried over from a `/compact` context summary. See `TUNING_LOG.md` for the
detailed tuning trail and data tables this summary references.

## What changed this session

- Fixed `left_turn()`: was pivoting the wrong physical direction (turned
  right instead of left). Fixed by swapping which wheel drives forward.
- Added a fixed 5-segment path routine (`PathStep[] PATH`, `spinInPlace()`,
  `runPathStepManeuver()`, state-machine `loop()`):
  straight 4.5 m → spin right 360° + turn right 90° →
  straight 5.7 m → spin left 360° + turn right 90° →
  straight 4.5 m → spin right 360° + turn right 90° →
  straight 5.7 m → spin left 360° + turn right 90° →
  straight 4.5 m → spin right 360° → stop.
- Field-calibrated `PULSES_PER_METER = 105.1` (measured: 450 cm run ≈ 473
  avg pulses; theoretical estimate 97.9 was off ~7%).
- Field-calibrated `SPIN_SLIP_FACTOR = 1.71` for point-turn tire scrub
  (tuned from real hardware: 1.0 → ~45° actual, 2.0 → ~105° actual, 1.71 →
  target 90°).
- Fixed `spinInPlace()` CW/CCW direction: geometric derivation was
  backwards on the physical robot, branches swapped.
- Fixed `spinInPlace()` missing `MOTOR_L_RATIO` trim (left wheel was
  overpowering during spins, throwing off straight-line driving after a
  spin too). Confirmed straight on hardware after the fix.
- Nudged `MOTOR_L_RATIO` from `0.825` → `0.82` after a reported left-drift
  starting around 80 cm into a straight run (Serial log showed right
  pulses progressively out-pacing left). **Not yet confirmed on hardware.**

## Current state

- `robotcurclerun.ino`: `MOTOR_L_RATIO = 0.82`, `PULSES_PER_METER = 105.1`,
  `SPIN_SLIP_FACTOR = 1.71`, `PATH[]` 5-segment route in place.
- `forward()` / `backward()` core auto-balance logic untouched this
  session.
- Include on line 2 uses an absolute path intentionally (relative include
  failed to compile in Arduino IDE) — do not revert to relative.

## Open / pending

- Retest `MOTOR_L_RATIO = 0.82` on real hardware for the left-drift-at-80cm
  symptom. If still veering left, nudge down further (e.g. 0.815). If it
  flips to veering right, nudge back up slightly (e.g. 0.822).
- Once confirmed, log the result as a new row in `TUNING_LOG.md`'s
  `MOTOR_L_RATIO` tuning trail.
- `PULSES_PER_DEGREE`/`SPIN_SLIP_FACTOR` are from visual angle estimates,
  not protractor-measured — revisit if heading error compounds visibly
  across the 5-segment route.
- Possible mechanical/voltage-sag causes of drift noted in
  `TUNING_LOG.md`, not yet ruled out.

## Working conventions for this project

- Derive motion constants from real hardware test data (Serial Monitor
  pulse counts, measured distance/angle), not pure theory. Theoretical
  estimates were consistently off from measured hardware behavior.
- Only `git commit`/`git push` when explicitly asked, typically after a
  fix is confirmed working on real hardware.
