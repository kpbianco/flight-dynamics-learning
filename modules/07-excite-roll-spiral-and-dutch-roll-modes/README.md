# P07 — Excite Roll, Spiral, and Dutch-Roll Modes

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 2:** Stability and modes  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?

## Physical mental model

P06 showed that a mode is recognized from its excitation, dominant observables, time scale, and
envelope—not from a plot color. P07 applies that method to three lateral-directional signatures:

```text
roll subsidence: p_dot   = -lambda_R p
                  phi_R_dot = p

spiral:          phi_S_dot = -lambda_S phi_S
                  psi_dot   ~= (g/V0) phi_S  (small coordinated-turn view)

Dutch roll:      beta_dot = -r              (nearly frozen flight path)
                  r_dot    = omega_D^2 beta - 2 zeta_D omega_D r
```

An idealized aileron pulse sets the initial roll rate, an initial bank release exposes the spiral,
and an idealized rudder pulse sets the initial yaw rate. Positive roll rate and bank are
right-wing-down; positive yaw rate and heading are nose-right; positive sideslip is air-relative
velocity toward body right. Therefore a positive nose-right yaw impulse initially drives sideslip
negative in the frozen-path approximation.

The roll mode is fast and aperiodic: roll rate fades, but its bank-angle integral need not return to
zero. Dutch roll is an oscillatory sideslip/yaw-rate exchange. The spiral is slow enough to appear
nearly harmless before a stable or unstable sign becomes obvious.

## Learning flow

1. Read the P06 connection and make one prediction about the three time scales.
2. Excite all three deterministic baseline modes and inspect one view at a time.
3. Sweep roll decay rate alone and observe roll-rate settling and integrated bank change.
4. Read the roll-moment, inertia, and damping mechanism, then reset.
5. Sweep Dutch-roll damping alone and observe sideslip-envelope retention per cycle; connect it to
   the separate modal-energy diagnostic.
6. Reverse the stable spiral sign and diagnose slow divergence beyond the linear bank limit.
7. Run independent checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P07")
run_module_checks("P07")
```

The implementation uses base MATLAB arithmetic and graphics. It contains no random input, file or
network I/O, numerical integrator, eigensolver, state-space helper, Simulink model, or identified
aircraft data. The pulse inputs are collapsed to angular impulses; the modes are deliberately
decoupled; Dutch-roll roll participation and full lateral sideforce/gravity coupling are omitted.
P08 later relates stability derivatives to coupled motion, and P09 later integrates six-degree-of-
freedom equations. This module is a modal learning approximation, not a handling-qualities or
flight-fidelity claim.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — guarded, bounded, closed-form modal calculations.
- `experiment.m` — baseline, two independent sweeps, metrics, and broken spiral sign.
- `interactive.m` — excitation, decay-rate, and damping controls with immediate plots.
- `lesson.md` and `walkthrough.md` — tutor explanation and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
