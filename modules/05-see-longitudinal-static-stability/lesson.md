# P05 lesson: See Longitudinal Static Stability

## Guiding question

What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?

## Compounds on P04

P04 used P03's density and true airspeed to obtain dynamic pressure, then found the lift coefficient
and angle of attack required for point-mass force trim. Its baseline gave
`q = 1325.0079853 Pa` and `alpha = 3.4175453 deg`, but it explicitly omitted pitching-moment and
elevator trim. P05 uses that air state as a reference and adds only incremental pitching moment.
For each CG and tail geometry, zero perturbation means a separately retrimmed local reference; it
does not claim that the old absolute elevator setting still trims after a geometry change.

## Mental model and signs

Measure every station `h = x/c_bar` aft from the MAC leading edge. Positive `C_m` and pitching
moment are nose-up. Positive elevator perturbation is trailing-edge down, which increases upward
tail lift in this model and therefore creates a nose-down moment because the tail is aft of the CG.

For a small angle-of-attack change, the transparent component buildup is

```text
K_t       = eta_t (S_t/S) a_t (1 - d epsilon/d alpha)
C_L_alpha = a_w + K_t
h_n       = (a_w h_w + K_t h_t) / C_L_alpha
SM        = h_n - h_cg
C_m_alpha = a_w(h_cg - h_w) + K_t(h_cg - h_t)
          = -C_L_alpha SM
```

The incremental moment then follows

```text
C_m_delta_e = -eta_t (S_t/S) a_t tau_e (h_t - h_cg)
delta C_m   = C_m_alpha delta alpha + C_m_delta_e delta elevator
delta M     = q S c_bar delta C_m
alpha_abs [deg] = alpha_ref,P04 [deg] + delta alpha [deg]
```

Angles multiplying coefficient derivatives are radians. The displayed P04 reference, perturbation,
and absolute alpha use degrees consistently; coefficient slopes use `1/rad`, static margin uses a
fraction or percent of MAC, and dimensional pitching moment uses `N*m`. The P04 reference alpha
stays fixed; `alpha_abs` is the disturbed absolute angle, not a new reference. The model keeps the
component form and the neutral-point form so their equality can be checked independently.

## Baseline, then one lever

The deterministic baseline uses `h_cg = 0.30`, `S_t/S = 0.20`, a `+2 deg` angle-of-attack
perturbation, and zero elevator perturbation. Visible teaching constants are `h_w = 0.25`,
`h_t = 3.5`, `a_w = 5.0 /rad`, `a_t = 4.0 /rad`, tail dynamic-pressure ratio `eta_t = 0.9`,
downwash gradient `0.4`, elevator effectiveness `0.6`, `S = 16.2 m^2`, and
`c_bar = 1.5 m`.

The resulting tail lift contribution is `0.432 /rad`, total lift-curve slope is `5.432 /rad`,
neutral point is `50.8468% MAC`, static margin is `20.8468% MAC`, and
`C_m_alpha = -1.1324 /rad`. The positive angle-of-attack disturbance produces
`delta C_m = -0.0395282` and `delta M = -1272.72 N*m`: nose-down and therefore restoring under the
declared sign convention.

First sweep CG while tail size and disturbance stay fixed. The aircraft geometry keeps the neutral
point fixed. Moving CG aft reduces static margin linearly, makes `C_m_alpha` less negative, reaches
neutral at the neutral point, and then produces a positive reinforcing moment.

Reset CG to `30% MAC`, then sweep horizontal-tail area. More tail area adds lift-curve slope far aft,
moving the neutral point aft and making the restoring slope more negative. With no tail, the neutral
point reduces to the wing aerodynamic center; at the baseline aft CG, the wing-alone slope is
positive and the elevator derivative is zero.

## Elevator is not the stability slope

At a fixed CG and tail geometry, elevator changes the moment intercept through
`C_m_delta_e delta elevator`; it does not change `C_m_alpha`. A control input can oppose an immediate
disturbance without changing the underlying stick-fixed static-stability derivative. Computing the
absolute elevator needed to retrim each geometry is outside this incremental lesson.

## Deliberately broken static-margin sign

Correct static margin is `h_n - h_cg`. The broken case computes `h_cg - h_n` but still inserts it
into `C_m_alpha = -C_L_alpha SM`. That reverses the baseline slope and turns the `+2 deg` response
from `-1272.72 N*m` nose-down to `+1272.72 N*m` nose-up. A polished line through the origin is not
evidence of stability; its slope and sign convention must agree with the physical first tendency.

## Scope and common misconceptions

- Static stability describes the initial moment tendency, not whether the later motion is well damped.
- Neutral means `C_m_alpha = 0`; it does not by itself prove absolute force or moment trim.
- Positive static margin places the CG ahead of the neutral point under the declared aft-positive station convention.
- Angle of attack is not pitch attitude, and an angle-of-attack perturbation is not an elevator command.
- More tail area can increase restoring tendency, but this lesson does not assess drag, structure, trim authority, or handling-quality tradeoffs.
- The linear derivatives and fixed constants are teaching approximations, not wind-tunnel, certification, or flight-test data.
- The bounded perturbation ranges keep the local linear picture visible; finite output is not a feasibility or fidelity claim.
- P06 adds time response, damping, short-period motion, and phugoid motion; P05 does not integrate dynamics.

## Completion standard

The learner can connect P04's air state to a moment scale; state the station and moment signs;
derive neutral point, static margin, and `C_m_alpha` from wing and tail contributions; predict the
isolated CG and tail-area effects; distinguish elevator response from the stability slope; diagnose
the reversed-margin failure; pass `run_checks.m`; and give the teach-back in `checks.md` without
relying on MATLAB syntax.
