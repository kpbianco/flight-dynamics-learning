# P05 — See Longitudinal Static Stability

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 2:** Stability and modes  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?

## Physical mental model

P04 balanced lift, drag, weight, and thrust at a deterministic air state but did not ask whether a
small pitch disturbance initially restores itself. P05 carries P04's dynamic pressure and required
angle of attack into a linear, stick-fixed moment model. Stations are measured aft from the mean
aerodynamic chord (MAC) leading edge and positive pitching moment is nose-up.

The wing and horizontal tail build a neutral point `h_n`. Static margin is
`SM = h_n - h_cg`, and the visible identity is `C_m_alpha = -C_L_alpha SM`. A forward CG gives
positive static margin and a negative slope: positive angle-of-attack disturbance creates a
nose-down restoring moment. At the neutral point the first-order tendency vanishes; aft of it, the
moment reinforces the disturbance.

## Learning flow

1. Read the station and pitching-moment sign conventions.
2. Visualize a deterministic baseline around P04's locally retrimmed reference.
3. Sweep CG position alone and watch static margin cross zero.
4. Read the lever-arm mechanism, then reset the CG.
5. Sweep horizontal-tail area alone and watch the neutral point move aft.
6. Reverse the static-margin subtraction and diagnose the reinforcing-moment symptom.
7. Run independent numerical checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P05")
run_module_checks("P05")
```

The implementation uses base MATLAB arithmetic and graphics. It contains no random input, file or
network I/O, optimizer, Simulink model, dynamic-mode solver, or opaque stability toolbox helper.
Every CG/tail configuration is an incremental, separately retrimmed reference; the model does not
predict the absolute elevator change required after moving the CG or resizing the tail.

## Files

- `lesson.m` — sectioned entry point and sign-first narrative.
- `model.m` — bounded deterministic derivative buildup and moment increments.
- `experiment.m` — baseline, CG and tail-area sweeps, metrics, and broken-sign case.
- `interactive.m` — CG, tail-area, angle-of-attack, and elevator controls.
- `lesson.md` and `walkthrough.md` — tutor explanation and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
