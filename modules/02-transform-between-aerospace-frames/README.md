# P02 — Transform Between Aerospace Frames

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 1:** Point-mass flight  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?

## Physical mental model

A velocity vector is one physical arrow, but its three coordinates depend on the axes used to describe it. This module starts with aircraft air-relative speed and the wind angles angle of attack and sideslip, forms body-axis components, and then uses roll, pitch, and yaw to express that same vector in right-handed North-East-Down coordinates. A valid direction cosine matrix preserves length; direction and known-sign cases reveal whether it was applied the right way.

## Learning flow

1. Read the body, wind, and North-East-Down conventions.
2. Visualize the deterministic baseline in body and NED views.
3. Sweep yaw alone and observe the North/East redistribution.
4. Reset, sweep sideslip alone, and observe body lateral velocity and track.
5. Read the matrix mechanism behind both changed views.
6. Deliberately transpose the forward transform and diagnose a direction error that a norm check misses.
7. Run independent numerical checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P02")
run_module_checks("P02")
```

The implementation uses base MATLAB only. Its fixed 3-by-3 calculations contain no random inputs, file I/O, network access, or toolbox frame-transform helpers. `velocityNed_mps` remains aircraft air-relative velocity expressed in NED axes; this module does not add atmospheric wind or compute ground velocity.

## Files

- `lesson.m` — sectioned entry point and concise concept narrative.
- `model.m` — deterministic wind-to-body and body-to-NED calculations.
- `experiment.m` — baseline, yaw and sideslip sweeps, metrics, and broken transform.
- `interactive.m` — speed, wind-angle, and attitude controls with immediate views.
- `lesson.md` and `walkthrough.md` — tutor dialogue and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
