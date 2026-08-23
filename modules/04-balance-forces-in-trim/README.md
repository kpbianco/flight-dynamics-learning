# P04 — Balance Forces in Trim

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 1:** Point-mass flight  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you balance Forces in Trim?

## Physical mental model

P03 turns altitude and temperature into air density, then combines density with true airspeed to
form dynamic pressure. P04 asks what angle of attack and thrust a notional aircraft would require
for a steady path. In path axes, with thrust aligned forward, force trim requires
`L = W cos(gamma)` and `T = D + W sin(gamma)`. The transparent aerodynamic model uses
`q = 0.5 rho V^2`, `CL = L/(q S)`, a linear lift law, and `CD = CD0 + k CL^2`.

The equations can always return a finite requirement inside the learning domain, but a requirement
is called feasible only when it stays below the declared `CLmax` and within the idealized constant
thrust cap. This is point-mass force trim, not pitching-moment, elevator, propulsion, or full-aircraft
trim.

## Learning flow

1. Read the path-axis force and sign conventions.
2. Visualize a deterministic level baseline using P03's standard 5 km density.
3. Sweep true airspeed alone and observe required lift coefficient, angle of attack, and the drag trade.
4. Read the dynamic-pressure and induced-drag mechanism.
5. Reset airspeed, sweep mass alone, and observe the added lift and thrust demand.
6. Omit the one-half in dynamic pressure and diagnose the resulting half-weight force residual.
7. Run independent numerical checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P04")
run_module_checks("P04")
```

The implementation uses base MATLAB arithmetic and graphics. It contains no random input, file or
network I/O, iterative trim solver, Simulink model, or opaque toolbox trim helper.

## Files

- `lesson.m` — sectioned entry point and physical narrative.
- `model.m` — deterministic analytic force balance, limits, and residuals.
- `experiment.m` — baseline, airspeed and mass sweeps, metrics, and broken dynamic-pressure case.
- `interactive.m` — density, true-airspeed, mass, and flight-path-angle controls.
- `lesson.md` and `walkthrough.md` — tutor explanation and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
