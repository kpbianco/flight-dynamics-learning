# Flight Dynamics and Aerospace GNC

A MATLAB-first, Khan-Academy-style learning track with 24 guided modules.

Each implemented module combines:

- a concise lesson and physical mental model;
- MATLAB `%%` notebook cells;
- deterministic plots;
- actual UI sliders, spinners, or dropdowns;
- two parameter sweeps;
- one deliberately broken case;
- executable numerical checks;
- a tutor protocol that asks one observation question at a time.

## Start

From a shell:

```bash
./bin/learn start
./bin/learn start P01
./bin/learn start P02
./bin/learn start P03
./bin/learn start P04
./bin/learn start P05
./bin/learn start P06
./bin/learn start P07
./bin/learn start P08
./bin/learn start P09
./bin/learn start P10
./bin/learn start P11
./bin/learn start P12
./bin/learn start P13
./bin/learn start P14
./bin/learn start P15
./bin/learn start P16
./bin/learn start P17
./bin/learn start P18
./bin/learn list
./bin/learn status
```

On Windows PowerShell:

```powershell
python .\bin\learn.py start
```

In MATLAB:

```matlab
launch_lesson("P01")
launch_lesson("P02")
launch_lesson("P03")
launch_lesson("P04")
launch_lesson("P05")
launch_lesson("P06")
launch_lesson("P07")
launch_lesson("P08")
launch_lesson("P09")
launch_lesson("P10")
launch_lesson("P11")
launch_lesson("P12")
launch_lesson("P13")
launch_lesson("P14")
launch_lesson("P15")
launch_lesson("P16")
launch_lesson("P17")
launch_lesson("P18")
run_module_checks("P01")
run_module_checks("P02")
run_module_checks("P03")
run_module_checks("P04")
run_module_checks("P05")
run_module_checks("P06")
run_module_checks("P07")
run_module_checks("P08")
run_module_checks("P09")
run_module_checks("P10")
run_module_checks("P11")
run_module_checks("P12")
run_module_checks("P13")
run_module_checks("P14")
run_module_checks("P15")
run_module_checks("P16")
run_module_checks("P17")
run_module_checks("P18")
```

`P01` remains the reference implementation; `P02` through `P18` are implemented lessons spanning
frame transforms, atmosphere, point-mass force trim, longitudinal static stability, and the
longitudinal and lateral-directional modes into nonlinear rigid-body propagation. P06 turns P05's restoring-moment slope into a transparent reduced-order
time response with explicit inertia and damping, separate fast/slow views, two independent damping
sweeps, and a broken damping sign. P07 contrasts fast roll subsidence, slow spiral motion, and
oscillatory Dutch roll using isolated pulse/release experiments, two independent shaping sweeps, and
a deliberately reversed spiral-stability sign. P08 traces nondimensional lateral stability
derivatives through dimensional loads and state-matrix entries into coupled sideslip, roll, yaw, and
bank-angle motion, with roll-damping and weathercock-stability sweeps plus a broken rate-normalization
case. P09 integrates NED position, body velocity, quaternion attitude, and body rates under transparent
force and moment pulses, with independent input sweeps and a broken rotating-frame transport term.
P10 models the transparent first-order actuator conceptually upstream of P09's internal moment
boundary, separates requested, feasible, and delivered deflection, sweeps lag and rate authority
independently, and exposes an omitted position envelope that invents pitch-moment authority. The lessons do
not provide a direct P10-to-P09 adapter. P11 turns prescribed pitch-rate, attitude, and NED
acceleration truth into gyro and body-frame accelerometer measurements. It independently sweeps
constant gyro bias and deterministic accelerometer-error vector RMS, then exposes a broken
specific-force equation that omits gravity. The curriculum connection from P10 through rigid-body
truth to P11 is conceptual; no direct adapter or estimator is included. P12 carries P11's declared
specific-force relationship into a fixed-attitude analytic body/NED trajectory. It independently
sweeps body-forward non-gravity specific force and heading, verifies frame-invariant speed, kinetic
energy, and power, closes mechanical-energy change to accumulated work, and exposes the false
energy residual created by treating positive NED Down as positive height. The P11-to-P12 link is
also conceptual; P12 does not consume P11 sensor histories or introduce estimation or control.
P13 begins the autopilot phase with a transparent cascaded altitude/pitch hold. It maps the P12
`h=-Down` convention through an outer altitude-to-pitch gain, a scheduled inner pitch response, a
separate first-order flight-path lag, and fixed-step altitude kinematics. Independent sweeps expose
outer-gain capture/overshoot/saturation and inner-loop tracking/control-demand trades. A broken
command-minus-measurement Down sign reverses only outer feedback; despite command saturation,
altitude error is still growing at the end of the fixed 30 s trace.
P14 compounds on P13's cascade pattern with circular heading outside a transparent second-order bank
response. It integrates continuous heading, wraps only display and feedback error, and uses the
coordinated level-turn relation `psi_dot=g*tan(phi)/V`. Independent sweeps expose heading-to-bank
gain/authority and roll-speed/acceleration trades. A broken raw subtraction interprets the
`+170 deg` to `-170 deg` command as `-340 deg`, saturates left bank, and is still turning the long
way at the end of the fixed `60 s` trace while independently computed shortest error grows.
P13 and P14 are fixed-speed reduced-order teaching systems, not P10/P12/P13 adapters, identified
aircraft, energy-closure models, or flight-control validation artifacts. P15 then makes true
airspeed a state: exact teaching-model drag feedforward plus speed-error corrective force commands
bounded thrust, a normalized first-order throttle lag delivers it, and thrust minus drag accelerates
the mass. Independent speed-gain and throttle-time-constant sweeps expose capture-authority and
delivery-rate trades. Reversed feedback commands idle after a positive speed step, so speed keeps
falling and proper error keeps growing through the fixed `30 s` horizon. P15 remains a straight,
level, fixed-condition teaching model—not an engine deck, P10 adapter, gain schedule, identified
aircraft, or flight-control validation artifact.
P16 closes the autopilot phase with a frozen-condition roll-loop gain schedule. True airspeed and
density form dynamic pressure, actual dynamic pressure scales transparent control effectiveness,
and a visible five-knot table linearly interpolates roll-angle and roll-rate gains. Independent
airspeed and density sweeps separate response consistency from equivalent-aileron demand. A broken
true-airspeed-only lookup is exposed with two conditions that have equal actual dynamic pressure:
the wrong lookup omits density, clamps to the high table endpoint, and produces a slower,
less-damped response even though the plant is unchanged. P16 is not a P14/P15 adapter, identified
gain schedule, robustness-margin result, certified controller, or aircraft validation artifact.
P17 begins navigation with a transparent one-dimensional INS/GPS alpha-beta teaching filter.
Gravity-compensated North acceleration drives a 50 Hz position/velocity prediction; one-Hz GPS
position innovations pass through an explicit inclusive gate before fixed-gain corrections.
Independent INS-bias and deterministic GPS-error sweeps separate integrated dead-reckoning drift
from measurement-error injection. A broken accept-all mode uses the identical sensor stream but
admits a fixed `+80 m` outlier, producing a causal position and velocity jump. P17 does not consume
P16/P12/P11 runtime arrays and is not an attitude mechanization, covariance filter, receiver,
navigation-integrity result, identified vehicle, or flight-validation artifact.
P18 turns an ideal P17-style North/East position into an ordered stationary-waypoint route. It uses
`atan2(Delta East,Delta North)` for course clockwise from North, a shortest circular course error,
a bounded fixed-speed course response, and an inclusive arrival circle. Independent arrival-radius
and response-gain sweeps separate switching geometry from turn authority; a deliberately swapped
North/East bearing sends the vehicle toward the wrong cardinal direction. P18 consumes no P17
runtime arrays and does not implement P19's moving-target pursuit, lead, or intercept problem.
Implemented modules always form a contiguous prefix;
`curriculum/modules.json` is authoritative as later governed batches advance that frontier.
Scaffolded modules remain intentionally non-runnable until their own bounded batch is complete.

## Module layout

```text
modules/01-example/
├── README.md
├── lesson.m
├── model.m
├── experiment.m
├── interactive.m
├── lesson.md
├── walkthrough.md
├── checks.md
└── run_checks.m
```

## Learning contract

The flow is always:

> question → mental model → baseline → manipulate levers → observe plots → break an assumption → explain → check → teach back

This repository is compatible with the same tutor/build split used by `dsp-radar_learning`.
