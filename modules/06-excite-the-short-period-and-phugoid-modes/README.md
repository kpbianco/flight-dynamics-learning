# P06 — Excite the Short-Period and Phugoid Modes

**Track:** Flight Dynamics and Aerospace GNC  
**Phase 2:** Stability and modes  
**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?

## Physical mental model

P05 established a negative stick-fixed `C_m_alpha`: after an angle-of-attack disturbance, the
baseline aircraft initially produces a restoring pitching moment. That is stiffness, not damping.
P06 adds declared inertia and damping assumptions so the response becomes visible in time.

The transparent teaching model keeps two underdamped second-order modes separate:

```text
short period:  theta_dot = q
               alpha_dot = q - gamma_dot ~= q  (fast, nearly frozen path)
               q_dot = -2 zeta_sp omega_sp q - omega_sp^2 alpha

phugoid:       u_dot = -2 zeta_ph omega_ph u - g gamma
               gamma_dot = (2g/V0^2) u
```

A fixed-duration elevator pulse supplies an angular impulse to the fast short-period coordinate.
An initial airspeed/energy displacement supplies the slow phugoid coordinate. The short-period view
emphasizes angle of attack and pitch rate over seconds; the phugoid view emphasizes speed,
flight-path angle, and altitude exchange over tens of seconds. The `alpha_dot ~= q` closure is the
frozen-flight-path short-period approximation, not exact kinematics: angle of attack, pitch
attitude, and flight-path angle are not interchangeable.

## Learning flow

1. Read why P05 restoring stiffness does not prove dynamic damping.
2. Excite both modes in a deterministic baseline and compare their time scales.
3. Sweep short-period damping alone and inspect the fast envelope.
4. Read the stiffness–inertia–damping mechanism, then reset.
5. Sweep phugoid damping alone and inspect the slow speed/path exchange.
6. Reverse the damping sign and diagnose growth despite unchanged restoring stiffness.
7. Run independent numerical checks and give a two-sentence teach-back.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P06")
run_module_checks("P06")
```

The implementation uses base MATLAB arithmetic and graphics. It contains no random input, file or
network I/O, optimizer, numerical integrator, Simulink model, Control System Toolbox state-space
helper, or identified aircraft data. It is a decoupled linear modal approximation for learning,
not a full longitudinal model, handling-qualities assessment, or flight-fidelity claim.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — bounded analytic modal calculations separated from presentation.
- `experiment.m` — baseline, two damping sweeps, metrics, and broken-sign case.
- `interactive.m` — elevator, airspeed, and two damping controls.
- `lesson.md` and `walkthrough.md` — tutor explanation and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent invariants.
