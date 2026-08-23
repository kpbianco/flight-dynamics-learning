# P14 lesson: Hold Roll and Heading

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Roll and Heading?

## Compounds on P13

P13 separated an autopilot into an outer objective and an inner attitude loop:

```text
outer altitude error -> pitch command -> inner pitch response -> flight path -> altitude
```

P14 carries that architecture sideways:

```text
outer heading error -> bank command -> inner bank response -> heading rate -> heading
```

The connection is conceptual rather than current API compatibility. P14 does not accept a P13
history, pitch state, altitude command, or controller output. It assumes level flight rather than
running P13 at the same time. P07's positive-bank/positive-heading convention and coordinated-turn
idea are also conceptual inputs, not a data adapter.

## Heading error lives on a circle

A linear state has one obvious subtraction. Heading does not: adding or subtracting `360 deg`
names the same direction. P14 uses the half-open interval `[-180,180)` for displayed heading and the
transparent operation

```text
wrap(x) = mod(x+pi,2*pi)-pi
```

The continuous heading state is never wrapped. Only its display and the feedback error are:

```text
psi_display = wrap(psi_continuous)
raw_error   = psi_command-psi_display
e_psi       = wrap(raw_error)
```

For the retained command, `psi_display=+170 deg` and `psi_command=-170 deg`. Raw subtraction gives
`-340 deg`, but the circular error is `+20 deg`. Both `-170 deg` and `+190 deg` describe the same
direction, so the nearest continuous target is `+190 deg`.

Exactly `180 deg` has two equally short directions. This lesson does not claim a unique sign at
that antipode. It also never differentiates wrapped display heading: the display jumps at the
branch cut, but the physical state and its rate remain continuous.

## The outer heading loop commands bank

The outer controller is

```text
phi_command = sat(K_psi e_psi,+/-12 deg)
```

`K_psi` has units `rad/rad`: it converts heading angle error into bank angle command. Positive bank
is right-wing-down. With the declared positive nose-right heading convention, a positive shortest
error must command positive bank. The bank-command envelope says that a large navigation error
cannot ask this teaching loop for unbounded bank.

At baseline, `K_psi=0.5 rad/rad`, so the `+20 deg` circular error asks for `+10 deg` bank. No state
has moved at the command sample; command must change before bank rate, bank, heading rate, and
heading can respond.

## The inner roll loop tracks bank

P14 represents the already-closed inner roll loop directly:

```text
phi_dot   = p_phi
p_phi_dot = omega_phi^2(phi_command-phi)-2 zeta_phi omega_phi p_phi
```

Here `zeta_phi=0.8` is fixed and `omega_phi` is the selected closed-loop natural frequency. The
state `p_phi` is bank-angle rate under the level-attitude approximation. It is not necessarily the
full-aircraft body-axis roll rate `p` when pitch, yaw, and Euler kinematic coupling are present.

The equation stays visible: a bank error creates bank acceleration, while bank rate creates
damping. No `pid`, transfer function, state-space helper, pole-placement routine, solver, or toolbox
call hides the operation. `omega_phi` is a teaching bandwidth, not P07's aerodynamic roll-
subsidence pole, identified control-law data, or a certified performance value.

## Bank changes heading through a coordinated turn

In a steady coordinated level turn, resolving lift vertically and horizontally gives

```text
L cos(phi) = W
L sin(phi) = m V psi_dot
```

Eliminating `L` yields

```text
psi_dot = g tan(phi)/V
```

P14 applies that relation instantaneously as a transparent reduced-order teaching approximation.
`tan` receives radians. For small bank, `tan(phi)≈phi`, recovering P07's
`psi_dot≈g phi/V` view. Positive bank produces positive heading rate; as heading error contracts,
the outer bank command returns toward zero and the modeled aircraft rolls level.

The relation assumes coordinated level flight, still air, and fixed `V=60 m/s`. With no wind,
heading and course coincide. A real aircraft must generate extra lift, coordinate rudder, manage
sideslip/adverse yaw, and couple roll, yaw, speed, altitude, and energy. P14 does not model those
tasks.

## Baseline: read the transition in order

The continuous heading begins at `+170 deg`. Displayed command remains there through the trim
segment, then changes to `-170 deg` at `1 s`. Baseline inputs are `K_psi=0.5 rad/rad`,
`omega_phi=2.4 rad/s`, and wrapped error mode.

Before the step, error, bank command, bank, bank rate, bank acceleration, and heading rate are
exactly zero. At the step, raw displayed subtraction is `-340 deg`, shortest error is `+20 deg`,
and bank command is `+10 deg`. Bank is still zero, so the inner equation initially produces
`+57.6 deg/s^2` bank acceleration while heading rate remains zero.

By `1.5 s`, bank is about `3.76565 deg` and heading rate is about `0.616362 deg/s`. At `10 s`,
shortest heading error is about `9.69340 deg`. The response reaches 90% of the intended turn
`27.2 s` after command and ends near `+189.873 deg` continuous heading, displayed as about
`-170.127 deg`. Final shortest error is about `0.126913 deg`, and peak bank is about
`9.69100 deg`. Those are deterministic reduced-order references, not flight-test requirements.

## Lever 1: heading-to-bank gain

Reset `omega_phi=2.4 rad/s` and wrapped error, then sweep
`K_psi=[0,0.25,0.5,0.75,1] rad/rad`.

- `K_psi=0` is the exact open-heading-loop limit. Command changes, but bank command, bank, heading
  rate, and continuous heading stay at trim. Final shortest error remains `20 deg`.
- Increasing gain asks for more bank from the same circular error. Error at `10 s` and 90% capture
  time fall monotonically across the retained sweep.
- Peak bank rises. At `K_psi=0.75` and `1`, the initial request exceeds `12 deg`, so the outer
  command saturates and extra gain cannot buy proportional initial bank.

Mechanism first: the outer gain changes how radians of navigation error become radians of bank
command. It does not change inner roll dynamics, damping, airspeed, gravity, or the turn relation.

## Lever 2: inner roll natural frequency

Reset `K_psi=0.5 rad/rad` and wrapped error, then sweep
`omega_phi=[1.2,1.8,2.4,3.0,3.6] rad/s`.

The outer conversion and command remain fixed. Increasing `omega_phi` moves bank farther by
`1.5 s` and lowers bank tracking RMS. Initial acceleration scales with `omega_phi^2`, so peak
reduced-order bank acceleration rises from `14.4` to `129.6 deg/s^2` across the sweep.

Mechanism first: inner-loop speed changes how rapidly bank follows its command, not how heading
error is measured or converted. Faster bank tracking costs acceleration demand. It does not imply
monotonically better final heading error, gain margin, actuator feasibility, or aircraft fidelity.

## Deliberately broken raw heading subtraction

The broken call preserves the grid, fixed values, gains, damping, command, initial state, limits,
roll equation, and turn equation. It changes only which already-computed heading error drives the
outer loop:

```text
correct: e_psi = wrap(-170 deg-(+170 deg)) = +20 deg -> right bank
broken:  e_psi =      -170 deg-(+170 deg)  = -340 deg -> left bank limit
```

Correct and broken state histories match through the command sample. At that sample, correct mode
commands `+10 deg` bank, while broken mode commands the `-12 deg` limit. The broken response then
travels about `116.105 deg` left over the fixed `60 s` horizon and ends with about `136.105 deg` of
independently computed shortest error.

The final retained second is not a recovery: the bank command remains saturated, continuous
heading moves about another `1.991 deg` left, proper shortest error grows by the same amount, and
terminal heading rate remains about `-1.991 deg/s`. This establishes continued wrong-way motion
through the observed horizon only; it is not an infinite-horizon route or stability proof.

The broken raw error appears to shrink because the controller is faithfully following a `-340 deg`
route. That does not make the route correct. The symptom is a bank command opposite the shortest
turn and growth of the independent circular error. This is not positive feedback: the chosen raw
error contracts. It is also not integrator windup—the model has no integral-of-error state—or an
actuator, sensor, wind, or Dutch-roll symptom, because none of those mechanisms exists here.

Command saturation and the fixed horizon keep the trace finite. They do not repair the topology
mistake or establish a safe route.

## Numerical and limiting invariants

- Every call retains 3001 samples, 3000 updates, and the same `0:0.02:60 s` grid.
- Command and every state/derivative remain exactly at trim before the step.
- Display heading and shortest error are independently reconstructed with base-MATLAB `mod`, not
  toolbox wrap helpers; the continuous heading recurrence never contains a branch cut.
- Every bank and heading update can be reconstructed from sample-`k` values.
- `psi_dot=g*tan(phi)/V` holds at every sample, with radians inside `tan`.
- Bank command never exceeds `12 deg`; all accepted-domain actual bank histories remain below the
  declared `15 deg` teaching envelope.
- Zero heading gain is the exact open-loop limit.
- Each five-point sweep changes one public lever while mode, command, grid, fixed constants, and the
  other lever remain unchanged.
- Correct and broken states match through command onset; only selected error calculation changes.
- Rejected inputs leave no persistent state; valid calls after rejection or broken mode reproduce
  baseline exactly.
- Eight accepted corners and a capped 18-case representative grid remain finite and fixed-size.

## Common misconceptions

- A jump in wrapped display heading is not a physical heading jump. Continuous heading and heading
  rate remain smooth.
- `-170 deg` is not always left of `+170 deg`; directions are cyclic.
- Bank angle is not heading rate. Bank creates lateral acceleration, and the declared turn relation
  maps it to heading rate.
- Heading is not course in wind. P14 equates them only because wind is absent.
- Higher gain is not free performance; bank-command saturation caps authority.
- Faster inner response is not free; acceleration demand rises with `omega_phi^2`.
- The bank-rate state is not a complete body-rate model outside the declared level approximation.
- A bounded bank command does not make a long-way route correct.
- The broken case is a circular-coordinate failure, not positive feedback or windup.
- P14 adds no integral action, anti-windup, actuator, sensor, estimator, wind, sideslip, adverse yaw,
  yaw damper, rudder coordination, speed control, altitude/energy closure, gain scheduling, fault
  tolerance, flight-envelope protection, hardware timing, or HIL behavior.

## Evidence boundary

Static source inspection and an independent standard-library Python equation oracle can establish
structure and simulated reference behavior. MATLAB syntax execution, MATLAB numerical behavior,
Live Editor order, figures, `uifigure` controls, callbacks, learner understanding, controller or
aircraft fidelity, bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, staging, and
production behavior require separate named evidence and are not implied here.
