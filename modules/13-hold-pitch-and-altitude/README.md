# P13 — Hold Pitch and Altitude

**Track:** Flight Dynamics and Aerospace GNC

**Phase 4:** Autopilots

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?

## Physical mental model

An altitude hold is a cascade, not one magic gain. The outer loop turns geometric-altitude error
into a pitch command. The inner loop turns pitch error and pitch rate into a bounded equivalent
pitch-control command. Pitch attitude changes first; a declared lift/path lag then changes flight-path
angle and climb rate:

```text
e_h       = h_command - h
theta_c   = sat(s K_h e_h, +/-10 deg)
u_theta   = sat(K_p(theta_c-theta) + K_ff theta_c - K_q q, +/-20 deg)
theta_dot = q
q_dot     = -a_theta theta - a_q q + b_u u_theta
gamma_dot = (theta-gamma)/tau_gamma
h_dot     = V sin(gamma)
```

Here `theta` and `gamma` are perturbations about level-trim pitch attitude and flight-path angle,
and `u_theta` is a generalized positive-nose-up pitch-control effect. It is deliberately not named
for a physical surface: earlier modules retain their own declared elevator convention. The sign
selector `s=+1` is correct negative feedback. The broken `s=-1` case reverses only the outer-loop
sign.

P12 established that geometric altitude is `h=-NED Down`. If `Down_command-Down_measured` is used
as though it were `h_command-h`, an upward command produces a negative pitch command: altitude falls,
the error grows, and the pitch command reaches its limit. This is the module's deliberately broken
case and a direct conceptual use of the prerequisite convention.

## Deterministic reduced-order experiment

The model starts at `1000 m`, straight and level, with fixed `60 m/s` airspeed. At `1 s`, altitude
command steps to `1030 m`. The fixed grid is `0:0.02:30 s`: exactly 1501 samples and 1500 explicit
forward-Euler updates. The baseline uses:

- outer altitude-to-pitch gain `K_h=0.004 rad/m`;
- inner pitch natural frequency `omega_n=2.4 rad/s` and damping ratio `0.8`;
- first-order pitch-to-flight-path time constant `1.5 s`;
- pitch-command and equivalent pitch-control limits of `10 deg` and `20 deg`.

At the retained baseline, pitch has moved about `2.59069 deg` by `1.5 s` while flight-path angle is
only `0.292885 deg`. That separation is deliberate: pitching the body is not instantaneous climb.
Altitude reaches 90% of the step `7.28 s` after command, overshoots by about `1.57559 m`, and ends
with about `0.01030 m` error. Peak pitch-control demand is about `3.30024 deg`.

These values are deterministic equation references from an independent standard-library Python
oracle. They are not MATLAB-runtime, MATLAB numerical-fidelity, graphics, UI, aircraft, bench, HIL,
or flight evidence.

## Two independent levers

1. Hold pitch natural frequency at `2.4 rad/s`, keep the feedback sign correct, and sweep
   `K_h=[0,0.002,0.004,0.006,0.008] rad/m`. At zero gain the altitude loop is open: pitch, path
   angle, control input, and altitude remain at trim despite the command. Increasing gain reduces
   early altitude error but increases overshoot; the largest values spend time against the
   `10 deg` pitch-command limit.
2. Reset `K_h=0.004 rad/m`, keep the sign correct, and sweep
   `omega_n=[1.2,1.8,2.4,3.0,3.6] rad/s`. The scheduled inner-loop gains hold damping ratio fixed.
   Higher natural frequency moves pitch farther by `1.5 s` and lowers pitch-command tracking RMS,
   while peak pitch-control demand rises. The path angle continues to lag pitch.

The interactive reset restores exactly `0.004 rad/m`, `2.4 rad/s`, and correct feedback between
lever experiments.

## Deliberately broken altitude/Down sign

The broken call `model(0.004,2.4,-1)` preserves the grid, command, gains, plant, limits, initial
state, airspeed, and path lag. Only the outer feedback sign changes. Correct and broken state
histories are identical through the command sample, while their command signals already have
opposite signs at that sample. The broken states then move nose-down for a positive altitude error.
By `30 s`, the reduced-order trajectory is near `728.84 m`, error exceeds `301 m`, and the pitch
command has been limited for more than 80% of retained samples.

Every history remains finite over the fixed `30 s` horizon. Static authority limits bound command
and descent rate, but they do not bound altitude error over an unlimited horizon or restore
stability. The recognizable symptom is error growth in the same direction as the correction, not an
actuator lag, wind gust, sensor-noise, or integrator-windup symptom.

## Scope and prerequisite boundary

P12 contributes the altitude/Down sign and frame meaning. P10 motivates an authority envelope, and
earlier longitudinal modules motivate stable pitch motion. P13 does not consume any prior module's
histories or expose an adapter to them. Its controller has no integral-of-error state or action. The
model has no estimator, sensor model, identified aerodynamics, actuator state/rate lag,
propulsion-energy coupling, speed control, turbulence, gain scheduling, full 6-DOF motion, or
certified flight-control law.

The first-order `gamma` response is an explicit pedagogical approximation for lift/path response at
fixed speed. It prevents equating pitch with flight path but does not establish aircraft fidelity.
Holding `60 m/s` during the climb assumes unmodeled propulsive/energy support; it neither inherits
P12 work-energy closure nor implements the P15 speed/throttle loop.
The static pitch-control limit is inactive over the accepted input domain and serves only as a
defensive bound; it does not reproduce P10 actuator dynamics. The active authority trade in this
lesson is the `10 deg` outer pitch-command limit.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P13")
run_module_checks("P13")
```

The implementation uses base MATLAB arithmetic, fixed arrays, an explicit bounded recurrence, and
`uifigure` controls. It does not call Control System Toolbox, an ODE solver, Simulink, random sources,
files, networks, devices, timers, futures, or parallel workers. There is no background computation
to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and concise cascade narrative.
- `model.m` — guarded deterministic controller, plant, limits, recurrence, and metrics.
- `experiment.m` — baseline views, two isolated sweeps, open-loop limit, and sign failure.
- `interactive.m` — two gain controls, sign-failure switch, exact reset, and immediate views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, mechanisms, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and the independent Python equation oracle can validate structure and
simulated reference behavior without MATLAB. They do not establish MATLAB execution, MATLAB
numerical behavior, Live Editor order, figures, callbacks, learner understanding, aircraft fidelity,
bench, HIL, field, release, deployment, or production evidence.
