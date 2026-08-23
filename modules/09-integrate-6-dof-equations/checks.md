# P09 checks: Integrate 6-DOF Equations

## Guiding question

What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?

Ask and answer one item at a time.

## Observation check

In the baseline, why do body velocity components `u`, `v`, and `w` change even after the forward
force and moment pulses end? Distinguish rotating body components from NED velocity, and point to the
force, gravity, and `-omega cross velocity` terms before referring to a plot.

## State-chain check

Trace the forward body-force pulse through `F_x/m`, body velocity, the body-to-NED matrix, NED
velocity, and North position. Then trace the moment pulse through angular momentum, body rate,
quaternion, the same matrix, and the translated path. Which variables are states, which are inputs,
and which are display-only Euler angles?

## Frame and sign check

Point body `x` forward, `y` right, and `z` down. Explain why positive yaw maps body-forward toward
East, positive pitch maps body-forward toward negative Down, and positive roll maps body-right toward
positive Down. Why does gravity use `C_body_to_ned'` while position uses `C_body_to_ned`?

## First-lever check

Reset moment scale to `1`, then increase forward-force pulse scale. Predict the directly changed
load component, final North position, final speed, quaternion, body rates, and moment history. Why can
the path change without any rotational history changing?

## Second-lever check

Reset force scale to `1`, then increase moment pulse scale. Predict peak body-rate magnitude, peak
attitude rotation, and East/Down displacement. Why can translation change even though the entire
body-force history remains fixed?

## Limiting-case and interpretation checks

- Why do zero force and moment pulse scales give exact `[60t,0,0] m` NED motion?
- With moment scale zero, what analytic impulse and North-position result does the half-sine forward
  force produce?
- Why are 13 numeric states compatible with six physical degrees of freedom?
- Why is quaternion norm one necessary but not sufficient for a physically correct trajectory?
- Why are body rates `[p,q,r]` not generally Euler angle rates?
- Why must `C_body_to_ned` be orthonormal with determinant `+1`?
- Why does a body-fixed `-m g` force stop balancing gravity after the attitude changes?
- Why can body-rate components change after applied moment becomes zero?
- Why should inertial angular momentum and rotational kinetic energy remain constant after the
  moment pulse, even though the body-frame components of angular momentum can move?
- Why must each half-sine load be reevaluated at all four RK4 stages rather than sampled once per step?
- What numerical purpose does post-step quaternion projection serve, and what modeling errors can it not fix?
- How does this complete rigid-body integrator extend P08 without yet implementing P10 actuators,
  P11 sensors, or P12's dedicated energy/frame validation lesson?

## Broken-case check

The deliberately broken propagation omits `-omega cross velocity` from the body-axis velocity
equation while preserving the same force, moment, quaternion, and rate histories. Explain why the
resulting path can be smooth yet separate by about `132.792 m`; why substituting it into the complete
equation leaves an `omega cross velocity` residual; and why correct and broken cases coincide exactly
when moment scale is zero. What mistaken frame assumption would lead someone to write the broken
equation?

## Range, malformed-input, recovery, and resource check

Explain why both pulse scales are bounded to `[0,1.5]`; why nonscalar, complex, `NaN`, `Inf`, and
out-of-range inputs fail before propagation; and why a valid call after rejection must reproduce the
baseline. The smallest positive double is also inside the accepted range even when its moment response
rounds to zero, so the normalized angular-momentum drift must return zero rather than `0/0`. The model
performs two synchronous 301-sample, 300-step trajectories per call and a capped nine-case representative
grid in checks. It uses no external state, file, network, asynchronous task, timer, or unbounded loop,
so timeout and cancellation are deliberately not runtime semantics of this API.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P09")
```

`run_checks.m` covers determinism; fixed shapes and resources; independent load, position,
translational, quaternion, and rotational equations; every projected RK4 step; known frame signs;
unit quaternion and proper DCM; the exact zero limit; the analytic force-only limit; post-pulse
rotational energy and inertial angular momentum; finite smallest-positive-scale behavior; two isolated
sweeps; four accepted corners; a capped nine-case grid; malformed-input rejection and recovery; and
the deliberately omitted transport term. All assertions must pass before learner completion.

## Teach-back

In two sentences: first trace body force and moment through body velocity, angular velocity,
quaternion attitude, and NED position; then explain how omitting `-omega cross velocity` can leave
the same loads and attitude yet produce a smooth trajectory that violates the complete equation.
