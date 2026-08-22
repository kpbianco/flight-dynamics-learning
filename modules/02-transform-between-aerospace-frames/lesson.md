# P02 lesson: Transform Between Aerospace Frames

## Guiding question

What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?

## Compounds on P01

P01 used speed and heading to draw a path in North/East coordinates. This lesson exposes the missing bridge: onboard velocities and forces are often described in aircraft body axes, while flight paths and navigation live in local North-East-Down (NED) axes.

## Mental model

Picture one arrow painted in space. Turning the coordinate grid changes the three numbers written beside the arrow, but not the arrow's length or physical direction.

This lesson uses right-handed axes:

- body: `x_b` forward, `y_b` right, `z_b` down;
- NED: North, East, Down;
- wind: the aircraft air-relative velocity is `[V, 0, 0]^T` before angle of attack and sideslip are applied.

The transparent calculation is

```text
v_body = C_wind_to_body(alpha, beta) v_wind
v_NED  = C_body_to_ned(roll, pitch, yaw) v_body
```

`C_wind_to_body = R_y(-alpha) R_z(beta)`. `C_body_to_ned` uses the aerospace 3-2-1 yaw-pitch-roll sequence. Positive angle of attack produces positive body-down `w`; positive sideslip produces positive body-right `v`; positive pitch sends a forward vector toward negative Down.

## Baseline, then one lever

The deterministic baseline uses `V = 70 m/s`, `alpha = 6 deg`, `beta = 0 deg`, `roll = 0 deg`, `pitch = 9 deg`, and `yaw = 30 deg`. It gives track `30 deg` and flight-path angle `+3 deg` because, in this simple aligned case, track equals yaw and climb angle equals pitch minus angle of attack.

First sweep yaw while everything else stays fixed. The North and East components rotate and track follows yaw, while speed and flight-path angle remain fixed. This is length preservation by an orthonormal direction cosine matrix, not acceleration.

Reset to the baseline, then sweep sideslip. Body lateral velocity follows `v = V sin(beta)`, and the attitude transform carries that lateral component into NED track. Sideslip is therefore not an extra yaw angle; it describes velocity relative to the body.

## Deliberately broken direction

The transpose of `C_body_to_ned` is its inverse: it maps NED coordinates back to body coordinates. The broken case feeds a body vector through that inverse as though it were the forward map. At a pure `+90 deg` yaw, the correct forward velocity points east and the broken result points west.

Both answers retain exactly the same speed. That is the trap: orthogonality, determinant, and norm checks can all pass for the wrong transform direction. A known-sign limiting case and a labeled direction contract are also required.

## Common misconceptions

- A coordinate transform does not physically rotate or accelerate the aircraft.
- NED Down is positive; a climbing velocity has a negative Down component and a positive flight-path angle.
- Yaw is body attitude, while track is the horizontal direction of velocity. They coincide only under special alignment conditions.
- `velocityNed_mps` is air-relative velocity expressed in NED axes. Atmospheric wind must be added before calling it ground velocity.
- Pitch at `+/-90 deg` does not make the matrix entries infinite, but roll and yaw cease to be a unique 3-2-1 description. The model stays inside the principal nonsingular chart.

## Completion standard

The learner can name the source and destination frames, predict the sign of a limiting case, explain both lever responses without MATLAB syntax, diagnose the transpose misuse, pass `run_checks.m`, and give the teach-back in `checks.md`.
