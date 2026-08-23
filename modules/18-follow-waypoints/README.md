# P18 — Follow Waypoints

**Track:** Flight Dynamics and Aerospace GNC

**Phase 5:** Navigation and guidance

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you follow Waypoints?

## Physical mental model

P17 ended with an estimated North position and velocity. P18 asks what guidance does with position:
given an ordered list of stationary North/East waypoints, which waypoint is active, what course
points toward it, how quickly can course change, and when is that waypoint close enough to advance?

The fixed-target direct-to geometry is explicit:

```text
Delta North = North_waypoint - North_estimate
Delta East  = East_waypoint  - East_estimate
chi_command = atan2(Delta East, Delta North)
e_chi       = wrap(chi_command - chi)
chi_dot     = sat(K_chi e_chi, +/-12 deg/s)
North_dot   = V_g cos(chi)
East_dot    = V_g sin(chi)
```

Course `chi` is measured clockwise from North. The model uses the exact simulated North/East
position as an ideal navigation estimate, holds groundspeed at `25 m/s`, and advances only the
active waypoint when its Euclidean range is less than or equal to the arrival radius. The model
keeps course continuous internally, wraps only the displayed course and feedback error, and stops
route propagation after final acceptance. Its fixed-size terminal hold is recorder bookkeeping,
not a claim that a constant-speed aircraft stops instantaneously.

## Deterministic baseline

The `0:0.1:100 s` grid has 1001 samples and 1000 allocated intervals. The five fixed waypoints are
`[North East]=[0 0; 400 0; 400 300; 100 300; 100 650] m`; their four leg lengths are
`[400 300 300 350] m`, so planned centerline distance is exactly `1350 m`. The largest allowed
arrival radius is `80 m`, less than half the shortest leg, so adjacent arrival circles cannot
overlap.

With `R=30 m`, `K_chi=0.8 1/s`, and correct bearing geometry, an independent standard-library
equation oracle gives capture times `[14.8,29.4,41.7,59.2] s` and capture ranges approximately
`[30,28.612679,29.413713,27.827714] m`. The recorder propagates `1480 m` before final acceptance;
course-error RMS is about `33.914338 deg`, and course rate is saturated for `223/592` active
samples. These are deterministic simulated references, not MATLAB-runtime, UI, navigation,
aircraft, bench, HIL, or field results.

## Two independent levers

1. Hold `K_chi=0.8 1/s` and correct bearing mode, then sweep arrival radius through
   `[10,20,30,50,80] m`. All cases complete at `[63.1,61.0,59.2,55.5,50.0] s`; flown distance
   falls from `1577.5 m` to `1250 m`. A larger circle switches sooner and cuts more of each
   corner. Earlier completion is not greater spatial accuracy: each waypoint is accepted farther
   away.
2. Reset `R=30 m`, keep correct bearing geometry, then sweep course-response gain through
   `[0,0.2,0.4,0.8,1.2] 1/s`. Zero gain is the exact open-course-loop limit: course stays North,
   the first target is captured, and the remaining route is not completed. Positive cases complete
   at `[65.6,60.0,59.2,59.1] s`; increasing gain initially improves turn capture, then demands the
   same fixed `12 deg/s` rate limit more often.

Changing arrival radius does not change course-response gain, route, speed, limits, or bearing
mode. Changing response gain does not change arrival geometry, route, speed, limits, or bearing
mode. Reset between sweeps so the two mechanisms remain identifiable.

## Limiting cases and bounds

- At `K_chi=0`, commanded bearing can change but actual course rate is exactly zero. North reaches
  `2500 m`, East and course remain zero, and only waypoint 2 is captured.
- When course already equals commanded bearing, wrapped error and commanded course rate are zero.
- A range exactly equal to `R` is accepted; the next representable value above `R` is outside.
- While propagation is active, every position increment has length `V_g dt=2.5 m`.
- At the saturated rate limit, the fixed-speed turn-radius scale is
  `V_g/chi_dot_max = 25/deg2rad(12)`, about `119.366 m`. This is a kinematic scale, not a bank or
  aerodynamic limit.

## Deliberately broken North/East bearing

Correct course clockwise from North requires `atan2(Delta East,Delta North)`. Broken mode swaps
the arguments while keeping the waypoints, initial position, speed, response gain, arrival-rule and
propagation equations, limits, and grid identical:

```text
correct: chi_command = atan2(Delta East,  Delta North)
broken:  chi_command = atan2(Delta North, Delta East)
```

The first target is due North. Correct mode commands `0 deg`; broken mode commands `+90 deg` East.
The broken trace captures no target waypoints, misses waypoint 2 by about `296.869 m` at closest
approach, and remains incomplete at `100 s`. A small controller-error metric would not rescue this
case: a controller can accurately follow the wrong geometric command. The violated assumption is
that the two components passed to `atan2` match the declared North/East course convention.

## Scope and prerequisite boundary

P17 supplies the conceptual position-estimate input, not compatible runtime arrays. P18 assumes
that estimate is exact so it can isolate stationary-waypoint selection, bearing, circular course
error, bounded response, and arrival logic. Earlier P14 course/heading ideas are also conceptual;
P18 does not run its bank dynamics.

P19 remains separate: P18 has no moving target, target velocity, line-of-sight rate, closing speed,
lead or intercept calculation, target prediction, proportional navigation, or pursuit-law tuning.
The model also omits navigation error, wind, sideslip, current, altitude, terrain, obstacles,
geodesy, path planning, fly-by turn anticipation, bank/roll/actuator dynamics, speed control,
sensor latency, uncertainty, envelope protection, integrity logic, identified aircraft behavior,
and certification.

## Run

From MATLAB at the repository root:

```matlab
launch_lesson("P18")
run_module_checks("P18")
```

The implementation uses base MATLAB arithmetic, fixed arrays, explicit loops, labeled plots, and
`uifigure` controls. It calls no waypoint, navigation, mapping, control, optimization, or aerospace
toolbox; uses no random source, file, network, device, timer, future, or parallel worker; and retains
no state. There is no background task to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — guarded route, bearing, course-response, position, and arrival calculations.
- `experiment.m` — deterministic baseline, two isolated sweeps, limits, and broken bearing case.
- `interactive.m` — radius/gain controls, bearing mode, exact reset, and immediate views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, observations, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and an independent Python equation oracle can establish structure and
simulated reference behavior without MATLAB. They do not establish MATLAB parsing or execution,
figures, callbacks, learner understanding, guidance or aircraft fidelity, hardware, HIL, field,
release, deployment, or production behavior.
