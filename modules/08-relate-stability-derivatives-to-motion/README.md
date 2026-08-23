# P08 — Relate Stability Derivatives to Motion

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 2:** Stability and modes  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?

## Physical mental model

P07 showed recognizable roll, spiral, and Dutch-roll shapes by prescribing separate modal equations.
P08 works in the opposite direction: start with a sideslip perturbation, let every stability derivative
contribute a force or moment, and observe the coupled motion that emerges.

```text
state                normalized rate       coefficient increment
[beta, p, r, phi] -> p-hat=p b/(2V0)  ->  CY, Cl, Cn
                     r-hat=r b/(2V0)

coefficient -> dimensional load -> acceleration -> next coupled state
CY -> Y=qbar S CY   -> beta_dot = Y/(mV0) - r + (g/V0) phi
Cl -> L=qbar S b Cl -> p_dot = L/Ix
Cn -> N=qbar S b Cn -> r_dot = N/Iz
                                      phi_dot = p
```

Positive `beta` is air-relative velocity toward body right, positive `p` and `phi` are
right-wing-down, and positive `r` is nose-right. The declared baseline combines negative dihedral,
roll-damping, and yaw-damping derivatives with positive weathercock stability. Those signs do not prove
coupled stability by themselves, so the checks also require every state envelope to contract late in
the sampled response at the accepted derivative corners. No single derivative is a mode: every state
feeds several derivative terms, and the resulting accelerations feed back into all four equations.

## Learning flow

1. Transfer P07's sign conventions and make one prediction about the initial roll and yaw accelerations.
2. Release a deterministic `+3 deg` sideslip and inspect sideslip, yaw rate, roll rate, and bank one view at a time.
3. Trace the same motion through roll- and yaw-moment contribution ledgers.
4. Sweep `C_l_p` alone and observe unchanged initial roll acceleration but smaller developed roll-rate and bank peaks.
5. Reset, sweep `C_n_beta` alone, and observe the direct yaw acceleration plus coupled sideslip-crossing time.
6. Omit `b/(2V0)` from `C_l_p` and diagnose a smooth but unit-inconsistent, excessively roll-damped answer.
7. Run independent checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P08")
run_module_checks("P08")
```

The implementation uses base MATLAB arithmetic, graphics, and a visible fixed-step RK4 recurrence.
It contains no random input, external I/O, Control System Toolbox state-space helper, eigensolver,
matrix exponential, ODE solver, or Simulink model. The coefficients are declared teaching values at
one fixed trim condition. The model omits control inputs, product of inertia, longitudinal coupling,
position, nonlinear attitude kinematics, and changing atmosphere. P09 later integrates complete
six-degree-of-freedom equations. P08 is not identified-aircraft, handling-qualities, or flight evidence.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — guarded derivative ledger, coupled state equations, and bounded RK4 propagation.
- `experiment.m` — deterministic baseline, two independent sweeps, metrics, and broken normalization.
- `interactive.m` — sideslip, roll-damping, and weathercock-stability controls with immediate views.
- `lesson.md` and `walkthrough.md` — tutor explanation and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
