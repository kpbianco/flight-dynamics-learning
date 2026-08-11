# P01 — Turn Bank Angle into a Flight Path

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 1:** Point-mass flight  
**Status:** implemented

## Guiding question

How do bank angle and airspeed determine turn rate, radius, and load factor?

## Physical mental model

Start from a concrete system, measurement, or decision. Change one parameter at a time and connect every visible change to a physical or computational cause.

## Required learning flow

1. Establish a deterministic baseline.
2. Show at least two complementary plots or views.
3. Expose meaningful parameters as MATLAB controls or clearly editable Live Editor variables.
4. Sweep two parameters independently.
5. Include one deliberately broken or misleading case.
6. Ask one observation question at a time.
7. Finish with a teach-back and a deterministic check.

## Implementation contract

The completed module owns these files:

- `lesson.m` — notebook-style MATLAB sections (`%%`) and concise narrative.
- `interactive.m` — `uifigure` controls, plots, and immediate feedback.
- `model.m` — deterministic calculations separated from presentation.
- `experiment.m` — reproducible baseline, sweeps, and broken case.
- `lesson.md` — tutor-facing explanation and misconceptions.
- `walkthrough.md` — expected observations in order.
- `checks.md` and `run_checks.m` — conceptual and numerical completion checks.

Prefer base MATLAB. Optional toolbox comparisons may be added only after the underlying operation is visible.
