# P06 lesson: Excite the Short-Period and Phugoid Modes

## Guiding question

What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?

## Compounds on P05

P05 used P04's `qbar = 1325.0079853 Pa` reference and showed that its baseline
`C_m_alpha = -1.1324 /rad` produces a nose-down restoring moment after positive `delta alpha`.
That result describes the first tendency. It does not say how quickly the aircraft responds, how
much it overshoots, or whether oscillations decay. P06 retains P05's reference static margin
`20.8468% MAC`, moment slope, and elevator derivative, then declares a teaching pitch inertia and
two damping ratios to make the missing time behavior visible.

## Two modes, two time scales

The reduced short-period coordinate uses

```text
theta_dot = q
alpha_dot = q - gamma_dot ~= q  when the fast mode nearly freezes gamma
q_dot     = -2 zeta_sp omega_sp q - omega_sp^2 alpha

M_alpha   = qbar S cbar C_m_alpha
omega_sp  = sqrt(-M_alpha/I_y)
q(0+)     = qbar S cbar C_m_delta_e delta_e Delta t / I_y
```

Positive elevator is trailing-edge down, positive pitch rate is nose-up, and the baseline uses a
`-2 deg` trailing-edge-up pulse for `0.18 s`. With `I_y = 1800 kg*m^2`, that pulse creates a
positive initial pitch rate. The `0.18 s` duration keeps every accepted normal pulse inside P05's
`+/-5 deg` local linear-alpha domain. The mode's natural frequency is `4.50066 rad/s`; at
`zeta_sp = 0.35`, its damped period is `1.49032 s`. Angle of attack moves quickly and pitch rate
leads the displacement. The closure `alpha_dot ~= q` is explicitly approximate: exact kinematics
give `alpha_dot = q - gamma_dot`.

The reduced phugoid coordinate uses

```text
u_dot     = -2 zeta_ph omega_ph u - g gamma
gamma_dot = (2g/V0^2) u
omega_ph  = sqrt(2) g/V0
h_dot     = V0 gamma
```

Here `u = delta V` is the airspeed perturbation and `gamma` is flight-path angle, not pitch attitude
or angle of attack. The baseline `V0 = 60 m/s`, `delta V(0) = +5 m/s`, and
`zeta_ph = 0.08` produce a `27.2703 s` damped period. The factor of two comes from lift's
`V^2` sensitivity in the constant-`C_L` point-mass approximation. The analytic altitude view
integrates `h_dot = V0 gamma`; the retained speed-equation residual checks
`u_dot + 2 zeta_ph omega_ph u + g gamma = 0`.

The phugoid period is about `18.30` times the short-period period. Mode names follow that time-scale
and observable behavior, not array order or plot color.

## Baseline, then one lever

The deterministic baseline uses elevator pulse `-2 deg`, airspeed kick `+5 m/s`, short-period
damping ratio `0.35`, and phugoid damping ratio `0.08`. Inspect the fast `delta alpha` view first,
then pitch rate. Only afterward inspect slow `delta V`, `gamma`, and altitude. This ordering prevents
a slowly changing speed trace from hiding the short-period motion on a shared time axis.

First sweep `zeta_sp` through `0.10, 0.20, 0.35, 0.50, 0.65` while both excitation inputs and
`zeta_ph` stay fixed. More damping removes more short-period amplitude per cycle. The P05 restoring
slope and declared inertia keep `omega_sp` fixed; the damped frequency shifts slightly because
`omega_d = omega_n sqrt(1-zeta^2)`. The entire phugoid response remains identical.

Reset `zeta_sp = 0.35`, then sweep `zeta_ph` through `0, 0.04, 0.08, 0.12, 0.20`. At zero damping,
the speed/path envelope does not decay. Positive damping reduces the remaining amplitude after each
slow cycle. The entire short-period response remains identical.

## Inputs excite; damping shapes

The two amplitude controls answer a different question from the damping sweeps. Setting the elevator
pulse to zero removes short-period `alpha` and `q` without changing the phugoid. Setting the airspeed
kick to zero removes phugoid speed, path-angle, and altitude changes without changing the short
period. Reversing either input reverses its owned response in this linear model.

The rectangular elevator pulse is collapsed to an angular impulse; actuator motion during the pulse
is not simulated. The airspeed input is an initial energy displacement, not an elevator command.
These simplifications make modal participation visible without pretending to derive a full aircraft
state-space model. P08 later connects stability derivatives to motion, and P09 later integrates the
full six-degree-of-freedom equations.

## Deliberately broken damping sign

The correct envelope contains `exp(-zeta omega_n t)`. The broken case uses
`exp(+zeta omega_n t)` while leaving P05's restoring stiffness and the oscillation frequency
unchanged. Over only `2.5 s`, the baseline short-period peak grows from about `1.256 deg` to more
than `97 deg`. That trace has deliberately left the local `+/-5 deg` linear domain: it is a clear
failure symptom, not a large-angle aircraft prediction. A negative static moment slope can coexist
with divergent motion when the damping sign is wrong; static stability alone is not dynamic
stability.

## Scope and common misconceptions

- `C_m_alpha < 0` supplies restoring stiffness in this approximation; it does not prove positive damping.
- Short period is identified by fast angle-of-attack/pitch-rate behavior; phugoid is identified by slow speed/path/altitude exchange.
- `alpha`, pitch attitude, and `gamma` are distinct; `alpha_dot ~= q` is only the frozen-path fast-mode approximation.
- Larger damping makes the envelope decay faster but also changes damped frequency; it does not change the declared natural frequency.
- A zero input produces no owned mode here because the modes are deliberately decoupled; a real aircraft input generally projects into several modes.
- These analytic responses use fixed time grids and notional inertia/damping. They are not identified aircraft data, a handling-qualities assessment, or flight evidence.

## Dependency and evidence boundary

The implementation uses base MATLAB element-wise arithmetic, plotting, and `uifigure` controls. It
does not call Control System Toolbox, Simulink, an ODE solver, a numerical eigensolver, external I/O,
or random state. Static source checks and an independent Python equation oracle can verify structure
and the declared equations. They do not prove MATLAB execution, MATLAB numerical fidelity, Live
Editor order, graphics, callbacks, learner understanding, bench, HIL, field, or production behavior.
