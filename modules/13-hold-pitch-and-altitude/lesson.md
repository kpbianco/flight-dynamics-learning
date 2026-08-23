# P13 lesson: Hold Pitch and Altitude

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?

## Compounds on P12

P12 made one sign non-negotiable:

```text
geometric altitude h = -NED Down
```

P13 puts that declaration inside feedback. A correct upward-altitude error is
`e_h=h_command-h`. If a controller instead computes `Down_command-Down_measured` and treats it as
altitude error, it gets `-e_h`. That is not a harmless coordinate choice; it reverses feedback.

```text
Down_command = -h_command
e_D          = Down_command-Down
e_h          = -e_D
```

The connection is conceptual rather than current API compatibility. P13 does not accept a P12
trajectory or energy ledger. P10's actuator lesson similarly motivates a limit, but P13 uses only a
static generalized control envelope and does not consume a P10 command history or actuator state.

## Why altitude hold is cascaded

Altitude does not respond directly to a pitch-control command. The visible path is:

```text
altitude error
    -> outer-loop pitch command
    -> inner-loop equivalent pitch-control command
    -> pitch rate and pitch attitude
    -> flight-path angle
    -> climb rate
    -> altitude
```

The outer controller is

```text
theta_c = sat(s K_h (h_command-h), +/-10 deg)
```

where `K_h` has units `rad/m`. Correct negative feedback uses `s=+1`: when commanded altitude is
above measured altitude, pitch command is positive. The limit says the cascade cannot request
unbounded pitch merely because altitude error is large.

The inner loop uses pitch error and pitch-rate damping:

```text
u_theta = sat(K_p(theta_c-theta) + K_ff theta_c - K_q q, +/-20 deg)
```

Positive `u_theta` means a generalized nose-up pitch-control effect. It is deliberately not named
for a physical surface because earlier modules retain their declared physical-elevator convention.

## The transparent pitch plant and gain schedule

The reduced-order pitch plant is

```text
theta_dot = q
q_dot     = -a_theta theta - a_q q + b_u u_theta
```

with `a_theta=0.8 1/s^2`, `a_q=1.2 1/s`, and `b_u=12 1/s^2`. For a selected unsaturated closed-loop
pitch natural frequency `omega_n` and fixed damping ratio `zeta=0.8`, the model computes

```text
K_p  = (omega_n^2-a_theta)/b_u
K_ff = a_theta/b_u
K_q  = (2 zeta omega_n-a_q)/b_u
```

Substitute the unsaturated controller into the plant and the terms remain visible:

```text
theta_ddot + 2 zeta omega_n theta_dot + omega_n^2 theta
    = omega_n^2 theta_c
```

No `pid`, transfer-function, state-space, pole-placement, or simulation toolbox call hides that
relation. Saturation can break the ideal unsaturated equation, which is why control demand and
saturation must be observed rather than assumed away.

## Pitch is not flight-path angle

Pitch attitude perturbation `theta` describes a change in the body's nose direction about level
trim. Flight-path-angle perturbation `gamma` describes a change in velocity direction. Angle of
attack and lift dynamics make them different in real flight. P13
keeps the distinction visible with the declared approximation

```text
gamma_dot = (theta-gamma)/tau_gamma
h_dot     = V sin(gamma)
```

where `tau_gamma=1.5 s` and fixed `V=60 m/s`. This is a first-order teaching surrogate for the
path response, not an identified aerodynamic model. At the altitude-command step, pitch begins
moving before `gamma`; altitude changes only after `gamma` produces climb rate.

Using `h_dot=V*sin(theta)` would erase that causal transition and teach the misleading idea that
pitch attitude is climb angle. A full aircraft model would replace this path lag with force,
angle-of-attack, speed, and energy dynamics; P13 deliberately does not claim that fidelity.
Holding speed fixed during a climb assumes unmodeled propulsive/energy support; it does not inherit
P12's work-energy closure or implement P15's future speed/throttle loop.

## Baseline: read the transition in order

The aircraft surrogate begins at `1000 m` with `theta=q=gamma=0`. Command remains `1000 m` through
the trim segment, then steps to `1030 m` at `1 s`. Baseline inputs are `K_h=0.004 rad/m`,
`omega_n=2.4 rad/s`, and `s=+1`.

Before the step, every error, command, state derivative, and state remains exactly zero. At the
step, altitude error becomes `+30 m`, so unclipped pitch command becomes `+0.12 rad` or about
`+6.87549 deg`. Pitch has not moved yet; the controller first commands about `+3.30024 deg` of
equivalent pitch-control effect. Pitch rate then grows, pitch attitude follows, `gamma` lags, and
altitude rises.

By `1.5 s`, pitch is about `2.59069 deg` while path angle is only `0.292885 deg`. The response reaches
90% of the altitude step `7.28 s` after command, overshoots about `1.57559 m`, and ends within about
`0.01030 m` of command at `30 s`. Those are deterministic reduced-order references, not flight-test
performance requirements.

## Lever 1: altitude-to-pitch gain

Reset the pitch natural frequency to `2.4 rad/s` and feedback sign to `+1`, then sweep
`K_h=[0,0.002,0.004,0.006,0.008] rad/m`.

- `K_h=0` is the exact open-altitude-loop limit. The command changes, but pitch command, control,
  pitch, path angle, climb rate, and altitude stay at trim. The final error remains exactly `30 m`.
- As `K_h` increases, the same altitude error produces a larger pitch request. Error at `5 s`
  decreases monotonically.
- The path lag carries the response after the error has begun shrinking, so overshoot increases.
- At high gain the pitch request reaches `10 deg`; increasing gain beyond that instant cannot buy
  proportional initial response because authority has clipped the request.

Mechanism first: outer gain changes how metres become radians. It does not directly change pitch
plant dynamics. The observed trade is faster early capture versus overshoot and time spent at the
pitch-command envelope.

## Lever 2: inner pitch natural frequency

Reset `K_h=0.004 rad/m` and `s=+1`, then sweep
`omega_n=[1.2,1.8,2.4,3.0,3.6] rad/s`.

The altitude command and outer conversion stay fixed. Increasing `omega_n` schedules the visible
pitch gains so pitch moves farther by `1.5 s` and pitch-command tracking RMS falls. That improvement
requires a larger peak pitch-control command. `gamma` continues to lag `theta` because the
path time constant did not change.

Mechanism first: inner-loop speed changes how rapidly control effect produces pitch, not how metres
are converted to commanded pitch. It trades tracking error against control demand.

## Deliberately broken altitude/Down feedback sign

The broken call keeps every fixed declaration and selected gain identical but sets `s=-1`. At the
positive altitude step:

```text
correct: theta_c = +K_h (h_command-h)  -> nose up
broken:  theta_c = -K_h (h_command-h)  -> nose down
```

The nose-down response reduces altitude. That makes `h_command-h` larger, which asks for still more
nose-down pitch. The pitch-command limit bounds command and descent rate over the retained horizon,
but the error grows to more than `301 m` by `30 s` and command remains limited for most samples.

Diagnose the loop direction before retuning gains. More inner-loop speed would make the wrong
correction happen faster. The symptom is not windup because this controller has no integral-of-error
state or action; it is not sensor noise because the measurement is exact; it is not P10 actuator lag
because no actuator state exists.

## Numerical and limiting invariants

- Every call retains 1501 samples, 1500 updates, and the same `0:0.02:30 s` grid.
- Command and all states remain exactly at trim before the step.
- `Down=-h`, `h_dot=V*sin(gamma)`, and every altitude, pitch, pitch-rate, and path-angle update can
  be independently reconstructed from the previous sample.
- Correct and broken state histories are identical through the command sample; their commands have
  opposite signs at that sample because only the declared outer feedback sign differs.
- Pitch command never exceeds `10 deg`; equivalent pitch-control command never exceeds `20 deg`.
- The `20 deg` pitch-control guard is inactive over the accepted domain; the observed authority
  trade is the active `10 deg` pitch-command envelope.
- Zero altitude gain is the exact open-loop limit.
- Each gain sweep changes one public lever while command, sign, resource count, and the other lever
  remain fixed.
- Rejected inputs leave no persistent state; a valid call after rejection reproduces baseline.
- Accepted corners and a capped representative grid remain finite and fixed-size.

## Common misconceptions

- Positive pitch is not automatically positive altitude; the flight path and climb rate must change.
- Pitch attitude and flight-path angle are not interchangeable.
- Higher gain is not free performance; it can increase overshoot, saturation, and control demand.
- A finite fixed-horizon trace can still reveal an unstable loop. Saturation bounds command and
  descent rate; it does not bound altitude error over an unlimited horizon or restore stability.
- Reversing both the coordinate and the error definition can preserve feedback; reversing only one
  of them changes negative feedback into positive feedback.
- The inner pitch loop must be interpreted inside the outer altitude loop; tuning either while
  ignoring authority and time-scale separation can mislead.
- The equivalent pitch-control sign is declared by its nose-up effect and is not a physical
  elevator-surface convention.
- The fixed-speed path lag is a transparent teaching approximation, not a trim, identified aircraft,
  full 6-DOF, or certification model.
- P13 adds no speed/throttle loop, integral action, anti-windup, gain scheduling, sensor fusion,
  fault tolerance, hardware timing, or HIL behavior.

## Evidence boundary

Static source inspection and an independent standard-library Python equation oracle can establish
structure and simulated reference behavior. MATLAB syntax execution, MATLAB numerical behavior,
Live Editor order, figures, `uifigure` controls, callbacks, learner understanding, controller or
aircraft fidelity, bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, staging, and
production behavior require separate named evidence and are not implied here.
