# P14 — Hold Roll and Heading

**Track:** Flight Dynamics and Aerospace GNC

**Phase 4:** Autopilots

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Roll and Heading?

## Physical mental model

Heading hold is a cascade. The outer loop turns a circular heading error into a bounded bank
command. The inner loop makes bank angle follow that command. In a coordinated, level, zero-wind
teaching turn, bank creates heading rate:

```text
wrap(x)     = mod(x+pi,2*pi)-pi       in [-pi,pi)
e_psi       = wrap(psi_command-wrap(psi_continuous))
phi_command = sat(K_psi e_psi,+/-12 deg)
phi_dot     = p_phi
p_phi_dot   = omega_phi^2(phi_command-phi)-2 zeta_phi omega_phi p_phi
psi_dot     = g tan(phi)/V
```

Positive bank is right-wing-down, and positive heading is nose-right. The continuous heading state
has no branch cut; only its display and feedback error are wrapped. The bank-rate state `p_phi` is
an Euler-angle rate in this declared level-turn approximation, not a complete body-axis roll-rate
model.

P13 introduced the same outer-objective/inner-attitude architecture for altitude and pitch. P14
changes the physical mechanism and adds a cyclic state: headings separated by `360 deg` describe
the same direction. The command from `+170 deg` to `-170 deg` is therefore a `+20 deg` shortest-path
right turn, not a `-340 deg` left turn.

## Deterministic reduced-order experiment

The fixed grid is `0:0.02:60 s`: 3001 samples and 3000 explicit forward-Euler updates. Heading
begins at `+170 deg`; at `1 s`, the displayed command changes to `-170 deg`. The baseline uses:

- heading-to-bank gain `K_psi=0.5 rad/rad`;
- inner roll natural frequency `omega_phi=2.4 rad/s` and damping ratio `0.8`;
- fixed true airspeed `60 m/s` and gravity `9.80665 m/s^2`;
- a `12 deg` bank-command limit and a checked `15 deg` bank teaching envelope;
- wrapped shortest-path heading error mode.

At the command sample, raw displayed subtraction is `-340 deg`, but wrapping produces `+20 deg`.
The outer loop commands `+10 deg` bank while heading, bank, and bank rate have not moved; the
reduced-order inner loop initially asks for `+57.6 deg/s^2` bank acceleration. By `1.5 s`, bank is
about `3.76565 deg` and coordinated heading rate is about `0.616362 deg/s`. Shortest heading error
is about `9.69340 deg` at `10 s`, reaches the 90% threshold `27.2 s` after command, and is about
`0.126913 deg` at `60 s`. Peak bank remains about `9.69100 deg`, below the declared envelope.

These values are independent standard-library Python equation references. They are not MATLAB
runtime, MATLAB numerical-fidelity, graphics, UI, aircraft, bench, HIL, or field evidence.

## Two independent levers

1. Hold roll natural frequency at `2.4 rad/s`, keep wrapped error, and sweep
   `K_psi=[0,0.25,0.5,0.75,1] rad/rad`. Zero gain is the exact open-heading-loop limit: the
   displayed command changes, but bank and continuous heading remain at trim. Higher gain reduces
   early error and capture time while increasing bank. At the two highest values, the `12 deg`
   command envelope becomes active.
2. Reset `K_psi=0.5 rad/rad`, keep wrapped error, and sweep
   `omega_phi=[1.2,1.8,2.4,3.0,3.6] rad/s`. Faster inner response moves bank farther by `1.5 s` and
   reduces bank-command tracking RMS, while peak reduced-order bank acceleration rises. The outer
   conversion, damping ratio, command, airspeed, and turn equation remain fixed.

The interactive reset restores exactly `0.5 rad/rad`, `2.4 rad/s`, and wrapped shortest-path error
between experiments.

## Deliberately broken heading subtraction

The broken call `model(0.5,2.4,0)` changes only the heading-error calculation. It raw-subtracts the
two displayed headings and treats the command as `-340 deg`. The outer command therefore saturates
at `-12 deg` bank and sends the modeled aircraft left. Correct and broken state histories match
through the command sample, but their bank commands already have opposite signs there.

Over the fixed `60 s` horizon, the broken case travels about `116.105 deg` left, finishes near
`+53.895 deg` displayed heading, and retains about `136.105 deg` of proper shortest heading error.
The bank command is saturated for more than 98% of all retained samples, while actual bank remains
inside the `15 deg` teaching envelope. During the final retained second, heading still moves about
`1.991 deg` left at about `-1.991 deg/s` and proper shortest error grows by the same amount. The
broken controller's raw arithmetic error appears to shrink along its chosen `-340 deg` route, but
the independent circular error grows. This establishes continued failure through the observed
horizon, not an infinite-horizon result. It is a coordinate/topology failure, not positive
feedback, wind, sensor noise, actuator lag, or integrator windup.

## Scope and prerequisite boundary

P13 contributes the cascade idea: an outer navigation objective commands an inner attitude loop.
P07 contributes the positive-bank/positive-heading convention and its small-angle
`psi_dot≈g phi/V` view; P14 uses `g tan(phi)/V` while keeping actual bank below `15 deg`. Those links
are conceptual. P14 does not accept prior histories, reuse a P13 controller, or expose an adapter.

The model assumes coordinated level flight, still air, exact states, fixed altitude, and fixed
`60 m/s` true airspeed. In that boundary, heading and course coincide. It omits sideslip, adverse
yaw, Dutch roll, wind, turn coordination by rudder, lift/load-factor and vertical dynamics,
actuator/sensor/estimator behavior, identified aerodynamics, gain scheduling, full 6-DOF motion,
and flight-envelope protection. P15 later studies speed/throttle; because heading rate scales as
`1/V`, P14's fixed speed is a declared teaching condition rather than a reusable speed-control
claim.

The selected `omega_phi` is a closed-loop teaching parameter, not P07's aerodynamic roll-subsidence
mode, a physical actuator command, an identified bandwidth, or certified control-law evidence.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P14")
run_module_checks("P14")
```

The implementation uses base MATLAB arithmetic, fixed arrays, explicit wrapping/saturation, a
bounded recurrence, and `uifigure` controls. It does not call Control System Toolbox, Mapping
Toolbox wrap helpers, an ODE solver, Simulink, random sources, files, networks, devices, timers,
futures, or parallel workers. There is no background computation to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and concise cascade narrative.
- `model.m` — guarded deterministic wrap, controller, roll response, turn kinematics, and metrics.
- `experiment.m` — baseline views, two isolated sweeps, zero-gain limit, and wrap failure.
- `interactive.m` — two lever sliders, raw-error switch, exact reset, and immediate views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, mechanisms, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and the independent Python equation oracle can validate structure and
simulated reference behavior without MATLAB. They do not establish MATLAB execution, MATLAB
numerical behavior, Live Editor order, figures, callbacks, learner understanding, controller or
aircraft fidelity, bench, HIL, field, release, deployment, or production evidence.
