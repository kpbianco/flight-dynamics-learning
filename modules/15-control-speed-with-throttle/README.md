# P15 — Control Speed with Throttle

**Track:** Flight Dynamics and Aerospace GNC

**Phase 4:** Autopilots

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you control Speed with Throttle?

## Physical mental model

P14 held heading while declaring true airspeed fixed at `60 m/s`. P15 makes that speed a state. A
speed controller asks for a bounded thrust change, normalized throttle delivers it with a
first-order lag, and the thrust-minus-drag residual accelerates the mass:

```text
e_V       = V_command - V
D(V)      = 0.5 rho S CD0 V^2 + 2 k W^2/(rho S V^2)
a_request = s K_V e_V
T_command = sat(D(V) + m a_request, 0, T_max)
delta_dot = (T_command/T_max - delta)/tau_T
T         = T_max delta
V_dot     = (T-D(V))/m
```

Correct feedback uses `s=+1`. Speed gain `K_V` has units `1/s`, so `K_V e_V` is a desired
acceleration. Multiplying by mass exposes the corresponding corrective force. The current drag
term is exact feedforward inside this teaching model: at zero error, commanded thrust balances
declared drag. It is not an identified feedforward schedule, engine deck, or claim that real drag
is known exactly.

The drag equation carries P04's level-flight parasite and induced terms conceptually. P10 contributes
the distinction between requested and delivered actuation, but P15 implements its own normalized
first-order throttle response rather than consuming a P10 history or adapter.

## Deterministic reduced-order experiment

The fixed grid is `0:0.02:30 s`: 1501 samples and 1500 explicit forward-Euler updates. True
airspeed starts at `60 m/s`; at `1 s`, its command changes to `70 m/s`. The baseline uses:

- speed gain `K_V=0.15 1/s`;
- throttle time constant `tau_T=0.8 s`;
- correct feedback sign `s=+1`;
- mass `1200 kg`, maximum thrust `4000 N`, and normalized throttle in `[0,1]`;
- P04-consistent `rho=0.736115547399152 kg/m^3`, `S=16.2 m^2`, `CD0=0.025`, `k=0.045`,
  `CLmax=1.4`, and `g=9.80665 m/s^2`.

At `60 m/s`, declared drag is about `826.952 N`, so delivered throttle begins at
`0.206738` and thrust exactly balances drag. At the command sample, the `+10 m/s` error requests
`+1.5 m/s^2` and commanded thrust jumps to about `2626.952 N`, or `65.674%` throttle. Delivered
throttle and acceleration are still at trim at that same sample; the state cannot respond before
the actuator moves.

At `1.5 s`, delivered throttle is about `0.416366` and acceleration is about
`0.697472 m/s^2`. Speed is about `64.071756 m/s` at `5 s`, error is about
`2.509388 m/s` at `10 s`, the first 90% capture occurs `14.34 s` after the command, and the
response enters and remains inside the `0.2 m/s` settling band at `23.68 s` after command. Final
error is about `0.079925 m/s`.

These values are independent standard-library Python equation references. They are simulated
reference outputs, not MATLAB runtime, MATLAB numerical-fidelity, plot, UI, aircraft,
propulsion-system, bench, HIL, or field evidence.

## Two independent levers

1. Hold `tau_T=0.8 s` and correct sign, then sweep
   `K_V=[0,0.075,0.15,0.225,0.3] 1/s`. Zero gain is the exact feedback-open trim limit: the
   command changes, but thrust continues to balance drag and speed remains `60 m/s`. Higher gain
   raises speed sooner and reduces error at `10 s`, while increasing delivered throttle demand.
   The highest gain activates the `4000 N` thrust-command limit.
2. Reset `K_V=0.15 1/s` and correct sign, then sweep
   `tau_T=[0.2,0.5,0.8,1.1,1.4] s`. Smaller time constants deliver more throttle and speed by
   `2 s` and reduce throttle tracking RMS, but peak normalized throttle rate rises. The gain,
   command, drag equation, mass, and thrust limit remain fixed.

The interactive reset restores exactly `0.15 1/s`, `0.8 s`, and correct feedback between
experiments. The time-constant sweep is interpreted through early response, request-delivery
mismatch, and throttle-rate demand; it does not claim monotonic final-error performance or engine
fidelity.

## Deliberately broken speed-feedback sign

The broken call `model(0.15,0.8,-1)` changes only the sign multiplying the already-computed
command-minus-speed error. Correct and broken speed and delivered-throttle histories match through
command onset. At that sample:

```text
correct:  D + m K_V(+10 m/s) = +2626.952 N -> 65.674% throttle command
broken:   D - m K_V(+10 m/s) =  -973.048 N ->  0.000% after saturation
```

The broken controller therefore commands idle. Delivered throttle decays rather than jumping, drag
exceeds thrust, speed falls, and the proper command-minus-speed error grows. Over the fixed horizon
of `30 s`, speed falls to about `40.9897 m/s`, proper error grows to about `29.0103 m/s`,
and the unclamped thrust request remains below zero for more than 96% of all retained samples.

During the final retained second, throttle command remains idle, speed falls by about `0.722 m/s`,
proper error grows by the same amount, and terminal acceleration remains below
`-0.72 m/s^2`. The trace remains about `3.44 m/s` above the declared `37.55 m/s` stall boundary.
This establishes continued failure through the observed horizon only. Saturation prevents
unphysical negative thrust, but it does not repair positive feedback. The model has no integral
state, so the symptom is not integrator windup.

## Scope and prerequisite boundary

P14 contributes the cascade idea and its fixed-speed boundary. P04 contributes the level-flight
drag decomposition; P10 contributes requested-versus-delivered actuation; P12 contributes the
forward-force and energy interpretation. These are conceptual links. P15 accepts no prior module
history, runs no earlier controller, and exposes no runtime adapter.

The model assumes exact true airspeed, straight level flight, constant mass, still air, fixed
density, constant lift equal to weight, thrust aligned with the flight path, linear
thrust-versus-throttle, exact knowledge of its drag model, and a fixed above-stall envelope. It
omits altitude and heading coupling, flight-path transients, lift/angle-of-attack dynamics,
propeller or jet maps, spool nonlinearities, fuel burn, wind, gusts, ground speed, sensors,
estimators, delays, discrete control, P/PI/PID implementation, anti-windup, identified
aerodynamics, full 6-DOF motion, envelope protection, and fault tolerance.

P16 later studies gain scheduling. P15's single fixed-condition `K_V` is not a schedule, a robust
controller, a stability-margin result, or certified control-law evidence.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P15")
run_module_checks("P15")
```

The implementation uses base MATLAB arithmetic, fixed arrays, explicit saturation, a bounded
recurrence, labeled plots, and `uifigure` controls. It does not call an ODE solver, Control System
Toolbox, Simulink, random sources, files, networks, devices, timers, futures, or parallel workers.
There is no background calculation to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and concise mechanism narrative.
- `model.m` — guarded deterministic drag, controller, throttle, force, speed, and metric logic.
- `experiment.m` — baseline views, two isolated sweeps, zero-gain limit, and reversed-sign failure.
- `interactive.m` — two lever sliders, feedback switch, exact reset, and immediate views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, mechanisms, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and an independent Python equation oracle can validate structure and
simulated reference behavior without MATLAB. They do not establish MATLAB parsing or execution,
MATLAB numerical behavior, Live Editor order, figures, callbacks, learner understanding, aircraft
or propulsion fidelity, bench, HIL, field, release, deployment, or production evidence.
