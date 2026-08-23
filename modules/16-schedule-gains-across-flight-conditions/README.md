# P16 — Schedule Gains Across Flight Conditions

**Track:** Flight Dynamics and Aerospace GNC

**Phase 4:** Autopilots

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?

## Physical mental model

P15 made true airspeed a state instead of a fixed label. P16 freezes one declared flight condition
per run, combines true airspeed and density into dynamic pressure, and uses that condition to select
roll-loop gains:

```text
qbar       = 0.5 rho V^2
sigma      = qbar/qbar_ref
b(sigma)   = b_ref sigma
delta_a    = sat(K_phi(sigma_s)*(phi_command-phi)-K_p(sigma_s)*p,
                 -delta_max,+delta_max)
phi_dot    = p
p_dot      = b(sigma) delta_a
```

`phi`, `p`, and the equivalent aileron command are positive right-wing-down. The physical plant uses
the actual ratio `sigma`. The lookup uses `sigma_s`, which may be actual dynamic pressure, the fixed
reference value, or—in the deliberately broken case—a true-airspeed-only surrogate that omits
density.

This is a transparent frozen-condition roll teaching model. Dynamic pressure scales only its
declared control effectiveness. It is not a P14 or P15 runtime adapter, an aerodynamic database, an
identified aircraft, a robustness result, or flight-control validation.

## Visible five-knot schedule

The reference condition uses `V_ref=60 m/s` and P15's declared
`rho_ref=0.736115547399152 kg/m^3`, so `qbar_ref=1325.0079853184736 Pa`. With
`b_ref=12 1/s^2`, target `omega_n=2.4 rad/s`, and target `zeta=0.8`, the table is:

| `qbar/qbar_ref` | `K_phi` (rad/rad) | `K_p` (s) |
| ---: | ---: | ---: |
| 0.50 | 0.960000 | 0.640000 |
| 0.75 | 0.640000 | 0.426667 |
| 1.00 | 0.480000 | 0.320000 |
| 1.25 | 0.384000 | 0.256000 |
| 1.50 | 0.320000 | 0.213333 |

At every knot, `b K_phi=omega_n^2` and `b K_p=2 zeta omega_n`. The model manually finds the two
ordered knots, computes a linear weight, and blends each gain. It exposes the raw lookup ratio,
clamped ratio, bracket indices, interpolation weight, selected gains, ideal analytic gains at both
the used lookup and actual plant condition, table interpolation error, and actual-condition gain
mismatch. Lookup values outside `[0.5,1.5]` hold the nearest endpoint rather than
extrapolate; the actual plant is never clamped. A clamp flag is a visible envelope warning, not
proof that the endpoint gains are safe outside the table.

## Deterministic baseline

The fixed grid is `0:0.01:8 s`: 801 samples and 800 forward-Euler updates. At `0.5 s`, roll command
steps from `0` to `10 deg`. The equivalent aileron command is bounded to `+/-15 deg`.

At the reference condition, scheduled and fixed-reference modes are exactly identical. The lookup
selects `K_phi=0.48 rad/rad` and `K_p=0.32 s`, so the onset command is `4.8 deg`. The deterministic
reference reaches the 90% band `1.23 s` after the step, enters and remains in the `0.2 deg` band at
`1.55 s`, overshoots by about `0.161546 deg`, and finishes with about
`-0.000006925 deg` error. No command saturation occurs.

These values come from an independent standard-library Python equation oracle. They are simulated
references, not MATLAB-runtime, UI, numerical-fidelity, aircraft, bench, HIL, or field evidence.

## Two independent levers

1. Hold density at `rho_ref` and sweep true airspeed through
   `[45,52.5,60,67.5,72] m/s`. The dynamic-pressure schedule keeps settling within
   `1.55–1.56 s`, while peak equivalent aileron falls from `8.8` to about `3.3536 deg`. Fixed
   reference gains expose the changing plant: the `45 m/s` case settles in about `3.29 s` with
   about `0.982 deg` overshoot, while the higher-pressure cases have different effective bandwidth
   and damping. Reset density and schedule mode before this sweep.
2. Reset airspeed to `60 m/s`, keep dynamic-pressure scheduling selected, and sweep density through
   `[0.5,0.75,1,1.25,1.5]*rho_ref`. These are exact table knots. All five roll histories overlay,
   while peak equivalent aileron falls from `9.6` to `3.2 deg`. The identical response does not mean
   condition is irrelevant; it is the intended consequence of changing gains with plant
   effectiveness.

The reference `qbar/qbar_ref=1` case is the limiting condition where scheduled and fixed gains are
exactly the same.

## Deliberately broken true-airspeed-only lookup

Dynamic pressure is not uniquely determined by true airspeed. Compare:

```text
reference: V=60 m/s, rho=rho_ref
paired:    V=75 m/s, rho=rho_ref*(60/75)^2
```

Both conditions have the same actual `qbar`, so correct scheduling produces the same plant, gains,
control history, and roll response. The broken mode instead uses `(V/V_ref)^2=1.5625`, omits density,
and clamps the lookup to `1.5`. It selects `K_phi=0.32` and `K_p=0.213333`, gains that are too small
for the unchanged reference-strength plant.

The broken response has effective `omega_n` about `1.959592 rad/s`, effective `zeta` about
`0.653197`, settling near `3.06 s`, and overshoot near `0.693006 deg`. Its selected angle and rate
gains are each one-third below the actual-condition ideal. The correct equal-pressure
case retains `2.4 rad/s`, `0.8`, `1.55 s`, and `0.161546 deg`. Only schedule selection changes; the
actual dynamic pressure, plant, command, grid, limit, and initial state do not. The clamp is part of
the symptom, not a repair.

## Scope and prerequisite boundary

P14 supplies the conceptual inner-roll-loop context. P15 supplies the lesson that true airspeed is a
physical quantity rather than a fixed plotting label. P03 supplies the density meaning. P16 accepts
no prior history, runs no earlier controller, and provides no adapter between module APIs.

The model omits Mach number, Reynolds number, altitude dynamics, mass, center of gravity,
configuration, aerodynamic cross-coupling, nonlinear control effectiveness, actuator dynamics and
rates, sensors, filters, schedule hysteresis, discrete flight-computer timing, turbulence, uncertainty,
stability margins, robustness analysis, full 6-DOF motion, envelope protection, fault tolerance,
certification, and operational constraints. Matching five knots and a bounded frozen-condition grid
does not prove stability between knots, outside the table, or on an aircraft.

## Run

After the manifest transition passes all governed checks, run from MATLAB at the repository root:

```matlab
launch_lesson("P16")
run_module_checks("P16")
```

The implementation uses base MATLAB arithmetic, fixed arrays, explicit saturation, a bounded
recurrence, manual interpolation, labeled plots, and `uifigure` controls. It does not call an ODE
solver, Control System Toolbox, Simulink, random sources, files, networks, devices, timers, futures,
or parallel workers. There is no background calculation to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — guarded deterministic condition, lookup, controller, plant, and metric calculations.
- `experiment.m` — baseline, airspeed and density sweeps, limiting cases, and broken lookup.
- `interactive.m` — true-airspeed and density controls, schedule selector, reset, and immediate views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, observations, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and the independent Python oracle can validate structure and simulated
reference behavior without MATLAB. They do not establish MATLAB parsing or execution, figures,
callbacks, learner understanding, controller or aircraft fidelity, hardware, HIL, field, release,
deployment, or production evidence.
