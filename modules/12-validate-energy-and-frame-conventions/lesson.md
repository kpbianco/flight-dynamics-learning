# P12 lesson: Validate Energy and Frame Conventions

## Guiding question

What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?

## Compounds on P11

P11 separated prescribed truth from measurements. Its ideal accelerometer relation was

```text
f_b = C_n_to_b (a_n-g_n)
```

where `f_b` is non-gravity specific force in body axes. P12 rearranges that relation:

```text
a_n = C_body_to_ned f_b + g_n
```

and asks whether the resulting velocity, position, power, work, and energy all agree. This also
revisits P09's body-to-NED state convention. The curriculum arrows P09 → P10 → P11 → P12 describe
conceptual order, not current API compatibility: P11 exposes sensor measurements on another fixed
history, while P12 accepts only a body-x specific-force magnitude and heading. There is no hidden
state adapter, estimator, controller, or closed-loop simulation.

## Declare the frames before trusting a number

The body frame uses `x` forward, `y` right, and `z` down. Navigation uses local
North-East-Down (NED). Positive pitch is nose-up, and positive heading turns clockwise from North
toward East. `C_body_to_ned` says what it does: if a vector has body components `a_b`, then

```text
a_n = C_body_to_ned a_b
```

For the fixed zero-roll attitude, the visible DCM is

```text
C_body_to_ned = [ cos(pitch)cos(heading), -sin(heading), sin(pitch)cos(heading)
                  cos(pitch)sin(heading),  cos(heading), sin(pitch)sin(heading)
                 -sin(pitch),              0,            cos(pitch)            ]
```

At positive pitch, the body-forward first column has a negative Down component. At heading
`+90 deg`, that same vector points toward positive East. Its transpose is the inverse only because
the matrix is a proper orthonormal rotation.

Orthonormality and determinant `+1` are necessary but not sufficient convention checks. A
self-consistent transpose or sign reversal may still preserve norms and round trips. That is why
the executable checks pin the signed `30 deg` pitch/heading column and a nonzero `2 s` velocity
sample against independent values.

## Kinematics and the P11 specific-force boundary

The audit uses constant attitude, initial body velocity `[60;0;0] m/s`, and

```text
f_b = [f_x;0;0] m/s^2
g_n = [0;0;9.80665] m/s^2
a_n = C_body_to_ned f_b + g_n
```

The analytic histories are

```text
v_n(t) = v_n(0) + a_n t
r_n(t) = r_n(0) + v_n(0)t + 0.5 a_n t^2
v_b(t) = C_body_to_ned' v_n(t)
```

No ODE solver or hidden integrator is needed. The `0:0.02:6 s` grid retains views and checks; it does
not approximate the trajectory. When `f_x=0`, the model is in free fall: `a_n=g_n`, ideal
accelerometer specific force is zero, and non-gravity work is zero. A supported sensor in P11 was a
different case because support supplied nonzero specific force.

## Frame-invariant kinetic energy and power

A proper rotation preserves dot products:

```text
v_b dot v_b = v_n dot v_n
f_b dot v_b = (C_body_to_ned f_b) dot v_n
```

Multiplying gives the same kinetic energy and non-gravity power in either frame:

```text
T       = 0.5 m (v dot v)
P_input = m f dot v
```

The components may look completely different while these scalar physical quantities stay the same.
That is the central frame check: a coordinate rotation can redistribute components, but it cannot
invent speed, kinetic energy, or power.

## Down is not altitude

The NED position state is `[North;East;Down]`. Geometric altitude is

```text
h = -Down
```

so uniform-gravity potential energy is

```text
U = m g h = -m g Down
E = T + U
```

With a non-gravity input, mechanical energy need not be constant. The correct invariant is the
work-energy balance:

```text
E(t)-E(0) = integral from 0 to t of m f_b dot v_b dt
```

The model evaluates the constant-acceleration work in closed form. `run_checks.m` independently
reconstructs every work interval with the trapezoidal rule; power is linear here, so the recurrence
is exact to roundoff.

## Baseline, then one lever

At fixed pitch `30 deg`, heading `30 deg`, and `f_x=1.5 m/s^2`, body-forward maps toward positive
North, positive East, and negative Down. The baseline reaches an apex gain of about `49.6872 m` at
`3.31248 s` and accumulates `537732.27 J` of non-gravity work by `6 s`. Mechanical-energy change
matches that work while body/NED kinetic energy and power agree.

First hold heading at `30 deg` and sweep `f_x=[0,0.75,1.5,2.25,3] m/s^2`. The DCM, initial
position, and initial velocity stay fixed. Increasing force increases input work, raises the apex,
and extends horizontal range. Mechanism-first explanation: the force does positive work on the
forward-moving body, and its nose-up orientation gives it a negative-Down component.

Then reset `f_x=1.5 m/s^2` and sweep heading through `[-90,-30,0,30,90] deg`. This is an active yaw
of the body and trajectory relative to fixed NED, not a passive coordinate transformation of one
unchanged trajectory. North and East histories rotate, while body velocity, Down/altitude, speed,
power, work, energy, and apex remain the same. Mechanism-first explanation: uniform gravity and no
wind or other horizontal asymmetry make the model yaw-symmetric, while the proper DCM preserves
norms and force-velocity dot products between body and NED descriptions.

## Limiting cases and invariants

- At `f_x=0`, ideal specific force, non-gravity power, and work are exactly zero; mechanical energy
  is constant while kinetic and potential energy exchange in free fall.
- At heading `0 deg`, East position, velocity, and non-gravity force are exactly zero.
- Headings `+90 deg` and `-90 deg` reverse only the East signs; their body histories and scalar
  ledgers match.
- Every accepted input retains 301 finite samples and 300 intervals. Input values cannot allocate a
  longer grid or launch asynchronous work.
- Every DCM is proper, body-to-NED-to-body round trips close, and body/NED speed, kinetic energy,
  and power agree.
- Mechanical-energy change matches independently accumulated non-gravity work.
- Rejected inputs leave no state; a valid call after rejection reproduces the baseline exactly.

## Deliberately broken Down-as-height convention

The broken ledger uses

```text
U_broken = +m g Down
```

as though positive Down were positive height. It keeps time, initial state, trajectory, DCM,
specific force, velocity, kinetic energy, power, and work identical. Therefore its false balance is
predictable:

```text
broken residual = 2 m g [Down(t)-Down(0)]
```

Both correct and broken residuals are zero at the initial datum. A single sample would miss the
defect. During the baseline climb, Down decreases and the broken ledger reports a large false energy
loss; the correct ledger continues to match work. The trace is smooth and finite because this is a
semantic sign failure, not numerical instability.

## Common misconceptions

- NED Down is positive, but altitude is its negative.
- Mechanical energy is constant only when non-gravity work is zero; with `f_x>0`, energy change
  should match work rather than stay zero.
- An accelerometer measures non-gravity specific force, not total coordinate acceleration.
- Free fall has `f_b=0` even though NED coordinate acceleration equals gravity.
- A DCM changes components, not a vector's physical norm or a force-velocity dot product.
- DCM orthonormality alone does not prove that its mapping direction or signs match the declaration.
- A smooth trajectory or small quaternion/DCM residual cannot validate the energy sign.
- The fixed attitude is a deliberate audit simplification, not a controlled aircraft maneuver.
- The forward specific force is a transparent teaching input, not a thrust, aerodynamic, actuator,
  or identified-vehicle model.
- P12 does not add rotation, drag, lift, propulsion dynamics, sensor fusion, feedback control,
  autopilot behavior, hardware timing, or HIL.

## Evidence boundary

Static source inspection and an independent standard-library Python equation oracle can establish
structure and simulated reference behavior. MATLAB syntax execution, MATLAB numerical behavior,
Live Editor order, figures, `uifigure` controls, callbacks, learner understanding, aircraft or
sensor fidelity, bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, and production
behavior require separate named evidence and are not implied here.
