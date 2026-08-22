# P03 — Build an Atmosphere Model

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 1:** Point-mass flight  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?

## Physical mental model

The atmosphere is the bridge between altitude and aerodynamic loading. A transparent dry-air model first maps positive-up geopotential pressure altitude to standard temperature and pressure. A local temperature departure then sets density through `rho = p/(R T)` and speed of sound through `a = sqrt(gamma R T)`. P02's air-relative speed magnitude completes the bridge to `Mach = V/a` and `q = 0.5 rho V^2`.

This lesson covers the gradient troposphere and isothermal lower stratosphere from 0 to 20 km. It does not silently extrapolate beyond that range. Pressure follows standard pressure altitude; the temperature-offset lever is a local air-state perturbation bounded to `-100` through `+100 K`, not a weather-column or geometric-altimetry model. True airspeed is bounded to `0` through `1000 m/s` so every accepted learning-model input produces finite outputs.

## Learning flow

1. Read the altitude, temperature, pressure, and airspeed conventions.
2. Visualize a deterministic 5 km baseline and its air-data metrics.
3. Sweep pressure altitude alone and observe the atmosphere and dynamic-pressure changes.
4. Read the hydrostatic, lapse-rate, ideal-gas, and speed-of-sound mechanism.
5. Reset altitude, sweep local temperature offset alone, and observe density, sound speed, Mach, and dynamic pressure.
6. Freeze density at its sea-level value at 11 km and diagnose the false aerodynamic-load scale.
7. Run independent numerical checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P03")
run_module_checks("P03")
```

The implementation uses base MATLAB formulas only. It contains no random input, file or network I/O, or opaque atmosphere-toolbox call.

## Files

- `lesson.m` — sectioned entry point and concept narrative.
- `model.m` — deterministic two-layer atmosphere and air-data calculations.
- `experiment.m` — baseline, altitude and temperature sweeps, metrics, and broken density case.
- `interactive.m` — altitude, temperature-offset, and true-airspeed controls with immediate views.
- `lesson.md` and `walkthrough.md` — tutor dialogue and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
