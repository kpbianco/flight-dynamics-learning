# P09 — Integrate 6-DOF Equations

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 3:** Six-degree-of-freedom simulation  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?

## Physical mental model

P08 mapped stability derivatives into force and moment tendencies for four lateral perturbation
states. P09 begins at the next boundary: given a body force and moment, advance all three
translational and all three rotational degrees of freedom while keeping attitude and position
consistent with the moving body frame.

```text
non-gravity force F_b       moment M_b
          |                      |
          v                      v
body velocity v_b          body rates omega_b
          |                      |
          |                quaternion q_body_to_ned
          +----------+-----------+
                     v
              NED position r_n
```

Body axes are `x` forward, `y` right, and `z` down. Navigation axes are North-East-Down (NED), so
Down is positive. The scalar-first unit quaternion and its direction cosine matrix rotate body
components into NED:

```text
v_n = C_body_to_ned(q) v_b

r_dot_n     = C_body_to_ned v_b
v_dot_b     = F_b/m + C_body_to_ned' [0; 0; g] - omega_b cross v_b
omega_dot_b = I \ (M_b - omega_b cross (I omega_b))
q_dot       = 0.5 q_body_to_ned tensor_product [0; omega_b]
```

The force input excludes gravity. A steady body `-z` force equals weight and balances gravity only
at the initial level attitude. Once a moment rotates the body, that same body-fixed force rotates in
NED and changes the trajectory. Euler roll, pitch, and yaw are derived for display; the integrator
propagates the quaternion, avoiding the 3-2-1 pitch singularity as an integration state.

## Deterministic experiment

The declared teaching body retains P08's `m = 1200 kg`, `I_x = 2500 kg*m^2`,
`I_z = 4000 kg*m^2`, and `u(0) = 60 m/s`, and adds `I_y = 3000 kg*m^2`. It begins level at the NED
origin with zero body rate. A `2400 N` forward half-sine force acts for `1.5 s`; a
`[500, 700, 350] N*m` roll-pitch-yaw half-sine moment acts for `1.0 s`. The visible fixed-step RK4
recurrence advances 13 numeric states on `0:0.02:6 s`, exactly 301 samples, with unit-quaternion
projection after every complete step.

At both pulse scales equal to `1`, the independent equation oracle gives a final NED position near
`[328.492, 28.830, 13.362] m`, final speed `43.250 m/s`, peak body-rate magnitude
`11.652 deg/s`, and peak attitude rotation `63.957 deg`. These values are deterministic teaching
references, not an identified aircraft trajectory or MATLAB-runtime evidence.

## Two independent levers

1. Sweep forward-force pulse scale through `0, 0.5, 1, 1.25, 1.5`, holding the moment pulse fixed.
   Only `F_x(t)` changes directly. Quaternion, body-rate, and applied-moment histories remain
   identical, while final North range and speed increase.
2. Reset force scale to `1`, then sweep moment pulse scale through `0, 0.5, 1, 1.25, 1.5`.
   The complete force history remains fixed. Angular impulse changes body rate and quaternion; the
   rotated body-fixed support force then changes East and Down motion.

At zero force and zero moment scales, the exact limiting case is straight level motion:
`r_n(t)=[60t,0,0]' m`, `v_b=[60,0,0]' m/s`, identity quaternion, and zero rates. With force scale
`1` and moment scale `0`, the half-sine pulse also has an analytic final speed and North position
against which the recurrence is checked.

## Deliberately broken transport term

Body velocity components are differentiated in rotating axes. Omitting
`-omega_b cross v_b` does not merely simplify the model; it changes the physical equation. The
broken comparison keeps time, loads, moments, quaternion, and body rates identical, but produces a
smooth wrong path. At the baseline it separates from the complete trajectory by about `132.792 m`
and leaves a peak translational closure residual above `9 m/s^2`. When moment scale is zero,
`omega_b=0`, so correct and broken cases coincide exactly—an important limiting check.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P09")
run_module_checks("P09")
```

The implementation uses base MATLAB arithmetic, graphics, a visible quaternion/DCM construction,
and a bounded synchronous RK4 recurrence. It does not call an ODE solver, Aerospace Toolbox,
Simulink, random source, file or network service, timer, or parallel worker. There is no background
task to time out or cancel. P10 adds actuator dynamics and limits, P11 adds sensors, and P12 performs
the dedicated energy and frame-convention lesson; none is silently absorbed here.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — guarded rigid-body equations, load definitions, RK4 propagation, and broken comparison.
- `experiment.m` — deterministic baseline, complementary views, two isolated sweeps, and broken case.
- `interactive.m` — force- and moment-pulse controls with immediate state and closure views.
- `lesson.md` and `walkthrough.md` — tutor explanation, prerequisite transfer, and observation order.
- `checks.md` and `run_checks.m` — interpretation questions and independent numerical invariants.

Static inspection and an independent Python equation oracle can validate structure and deterministic
reference behavior without MATLAB. They do not establish MATLAB execution, Live Editor order,
graphics, UI callbacks, MATLAB numerical fidelity, instructional effectiveness, bench, HIL, field,
or production evidence.
