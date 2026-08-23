# P09 lesson: Integrate 6-DOF Equations

## Guiding question

What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?

## Compounds on P08

P08 followed sideslip and body rates through stability-derivative contributions, dimensional force
and moment, acceleration, and a four-state linear response. It deliberately omitted complete
longitudinal/lateral coupling, nonlinear attitude kinematics, velocity, and position. P09 keeps the
same right-handed body convention—`x` forward, `y` right, `z` down—and supplies the missing rigid-body
propagation. P08 can eventually provide loads to this boundary; this lesson uses declared pulses so
the integrator remains visible without inventing a complete aerodynamic model.

The navigation frame is local North-East-Down (NED), with Down positive. A scalar-first Hamilton
quaternion `q_body_to_ned=[q0,q1,q2,q3]'` rotates body components into NED. This mapping statement is
more important than the subscript: if `a_b` is a body vector, then
`a_n=C_body_to_ned(q) a_b`.

## One state, four coupled operations

The 13 numeric states support six physical degrees of freedom:

```text
x = [north, east, down, u, v, w, q0, q1, q2, q3, p, q, r]'

position_dot_n = C_body_to_ned velocity_b

velocity_dot_b = force_non_gravity_b/m
               + C_body_to_ned' gravity_n
               - omega_b cross velocity_b

omega_dot_b    = I \ (moment_b - omega_b cross (I omega_b))

quaternion_dot = 0.5 q_body_to_ned tensor_product [0; omega_b]
```

Position needs the body-to-NED matrix because velocity components live in body axes. Gravity needs
its transpose because `[0,0,g]'` starts in NED and must enter the body velocity equation. The
`-omega cross velocity` term appears because the body basis rotates while its velocity components
are differentiated. The matching rotational cross product says a torque changes angular momentum,
not each body-rate component in isolation.

The quaternion derivative expands visibly to:

```text
q0_dot = 0.5 (-q1 p - q2 q - q3 r)
q1_dot = 0.5 ( q0 p + q2 r - q3 q)
q2_dot = 0.5 ( q0 q + q3 p - q1 r)
q3_dot = 0.5 ( q0 r + q1 q - q2 p)
```

The model normalizes the local quaternion used at every derivative evaluation and projects the
completed RK4 step back to unit length. Every resulting `C_body_to_ned` is independently checked for
orthonormal columns and determinant `+1`. Roll, pitch, and yaw are derived only for readable plots;
they are not integrated states.

## Declared load experiment

The initial state is level at the NED origin, flying body-forward at `60 m/s`, with zero body rate.
The body has `m=1200 kg` and diagonal inertia
`I=diag([2500,3000,4000]) kg*m^2`. Its non-gravity force is:

```text
F_b(t) = [2400 force_scale h(t,1.5); 0; -m g] N
```

and its applied moment is:

```text
M_b(t) = moment_scale [500;700;350] h(t,1.0) N*m
```

where `h(t,T)=sin(pi t/T)` inside the pulse and zero outside it. The `-m g` body-z force balances
weight only at the initial level attitude. Once the body rotates, that body-fixed force rotates in
NED; it is not a trim controller and does not silently follow the gravity vector.

The fixed-step recurrence reevaluates loads and equations at every RK4 stage:

```text
k1 = f(t_k, x_k)
k2 = f(t_k+dt/2, x_k+dt k1/2)
k3 = f(t_k+dt/2, x_k+dt k2/2)
k4 = f(t_k+dt,   x_k+dt k3)
x_(k+1) = x_k + dt (k1 + 2 k2 + 2 k3 + k4)/6
```

Here `dt=0.02 s`, the horizon is `6 s`, and exactly 301 samples are retained. The computation is
synchronous, stateless, and bounded: it has no timeout or cancellation API because it launches no
background work.

## Baseline, then one lever

With both pulse scales equal to `1`, the reference recurrence reaches NED position
`[328.49161, 28.82972, 13.36210] m`, final inertial speed `43.24982 m/s`, peak body-rate magnitude
`11.65232 deg/s`, and peak shortest attitude rotation `63.95728 deg`. After the moment pulse ends,
no torque remains; rotational kinetic energy and inertial angular momentum remain constant within
the accepted numerical tolerance. These values are simulated oracle outputs, not MATLAB execution.

First sweep force scale through `0, 0.5, 1, 1.25, 1.5`, holding moment scale at `1`. Only the
forward force history changes. Angular velocity and quaternion histories are therefore identical;
final North position and speed rise with forward impulse. This is direct isolation even though the
translated path is already coupled to the fixed attitude history.

Reset force scale to `1`, then sweep moment scale through `0, 0.5, 1, 1.25, 1.5`. The entire force
history now stays fixed. Larger angular impulse produces larger peak body rate and attitude rotation.
That attitude rotates both body velocity and the body-fixed support force, so East and Down position
also change: a rotational input can alter translation without changing its body-force input.

## Limiting cases

- At force scale `0` and moment scale `0`, the support force cancels gravity, rates remain zero,
  `C_body_to_ned=I`, and the exact result is `[north,east,down]=[60t,0,0] m`.
- At force scale `1` and moment scale `0`, the path stays North-only and the half-sine force has an
  analytic impulse. The final speed is `60 + 2(2400/1200)(1.5)/pi = 61.9098593 m/s` and final North
  position is `370.0267614 m`, within fixed-step tolerance.
- With moment scale `0`, `omega=0`; the transport term is exactly zero, so the complete and broken
  propagators coincide. A term can be essential generally and disappear in a valid limit.
- After the one-second moment pulse, `M_b=0`. Angular velocity components need not remain constant,
  but inertial angular momentum and rotational kinetic energy must.

## Deliberately broken rotating-frame equation

The broken comparison calculates

```text
velocity_dot_b(broken) = F_b/m + C_body_to_ned' gravity_n
```

and omits `-omega cross velocity`. The force, moment, quaternion, and rate histories remain identical
to the complete run, so this is an isolated failure. The wrong body-velocity components are then
mapped into a wrong NED trajectory. At the baseline, the final paths separate by `132.79178 m`, and
substituting the broken derivative into the complete equation exposes a peak residual of
`9.78153 m/s^2`. The plot stays smooth; equation closure and the zero-rate limiting case reveal the
failure.

## Common misconceptions

- Six degrees of freedom means three translations and three rotations. Quaternion attitude adds
  four numeric components with one unit-norm constraint; it does not add a seventh physical DOF.
- Body velocity is not NED velocity. Multiplying by `C_body_to_ned` is required before integrating
  NED position.
- Applied force here excludes gravity; adding gravity both to `F_b` and separately would count it twice.
- A body-fixed `-m g` force balances weight only while the body is level.
- `-omega cross velocity` is a coordinate-transport term, not drag or an aerodynamic derivative.
- `omega cross (I omega)` is present even with diagonal inertia; different principal inertias couple rates.
- Quaternion projection controls numerical norm drift. It does not validate the force or moment model.
- Euler angles are a display derived from the quaternion and may wrap; their components are not body rates.
- A smooth, finite trajectory is not evidence that frames, signs, or units are correct.
- The prescribed pulses are transparent teaching loads, not actuator, aerodynamic, or identified-aircraft fidelity.

## Evidence boundary

Source structure and a pure-standard-library Python implementation of the equations can establish
static and simulated reference evidence. MATLAB, Live Editor order, figures, `uifigure` controls,
callbacks, MATLAB numerical fidelity, learner understanding, bench, HIL, field, signing, deployment,
and production behavior require separate execution and retained evidence; none is implied here.
