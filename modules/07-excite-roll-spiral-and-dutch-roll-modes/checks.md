# P07 checks: Excite Roll, Spiral, and Dutch-Roll Modes

## Guiding question

What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?

Ask and answer one item at a time.

## Observation check

Use the baseline `0.4 s` roll time constant, `5.55436 s` Dutch-roll period, and `40 s`
spiral time constant to order the three signatures. Name the dominant observable pair or state for
each without referring to plot color.

## Excitation check

- Set the aileron pulse to zero. Which fast outputs vanish, and why do spiral and Dutch roll remain unchanged?
- Reset, then set bank release to zero. Which slow outputs vanish, and why is bank release an initial condition rather than a surface pulse?
- Reset, then set the rudder pulse to zero. Which oscillatory outputs vanish?
- Reverse one input at a time. Which states reverse, and why does Dutch-roll modal energy not change sign?

## First-lever check

At fixed aileron, bank release, rudder, spiral decay, and Dutch-roll damping, increase
`lambda_R`. Predict `p(0+)`, time constant, 2% settling time, integrated bank change, and both
unaffected modes. Explain why the moment impulse and inertia hold the initial rate fixed.

## Second-lever check

Reset `lambda_R = 2.5 1/s`, then increase `zeta_D` alone. Predict the Dutch-roll
envelope ratio, damped period, modal-energy trend, and the complete roll/spiral response. Explain the
zero-damping limit before positive damping.

## Limiting-case and interpretation checks

- Why does `p_dot + lambda_R p = 0` coexist with a nonzero final roll-mode bank increment?
- Why must the bank integral satisfy `phi_R_dot = p`?
- At `lambda_S = 0`, why does bank remain constant while the proxy heading changes linearly?
- Why does `omega_d^2 + (zeta_D omega_D)^2 = omega_D^2` hold?
- Why do `beta_dot + r = 0` and `r_dot - omega_D^2 beta + 2 zeta_D omega_D r = 0` belong together in this approximation?
- Why does the sampled `E = 0.5(r^2 + omega_D^2 beta^2)` history satisfy
  `E_dot = -2 zeta_D omega_D r^2`, and why is that diagnostic not energy in joules?
- Why can a stable spiral leave a finite heading change?
- Why is `psi_dot ~= g phi/V0` not a complete spiral eigenvector?
- Why can positive rudder initially produce negative sideslip under the declared conventions?
- Why do decoupled analytic modes fail to prove a real aircraft's handling qualities?

## Broken-case check

The deliberately broken case changes `exp(-lambda_S t)` to `exp(+lambda_S t)` with the same
initial `5 deg` bank. Explain why the trace reaches `100.428 deg` at `120 s` and why the
part after `15 deg` cannot be interpreted as a large-bank aircraft prediction. “The sign is wrong”
is incomplete: name the stable baseline assumption, slow observable, first sampled limit-crossing
time, and the fact that real aircraft can have a weakly unstable spiral.

## Range and transfer check

Explain why the model bounds pulse commands, bank release, decay rates, and damping; uses fixed
251-, 501-, and 481-sample grids; reports an infinite time scale for a neutral or numerically
unrepresentable near-neutral spiral; and keeps an unstable sign outside the normal API. Then
identify what P08 must add before derivatives can predict
coupled lateral motion and what P09 must add before six-degree-of-freedom integration is present.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P07")
```

`run_checks.m` covers determinism, fixed finite vectors, 64 accepted corners, independent moment-
impulse buildup, closed-form states, roll/bank and spiral/heading kinematics, Dutch-roll state and
nonzero energy-dissipation identities, time-scale separation, zero-input isolation, sign symmetry, neutral and
subnormal near-neutral spirals, undamped Dutch roll, both isolated sweeps, malformed inputs,
recovery, and the broken spiral sign.
All assertions must pass before learner completion.

## Teach-back

In two sentences: first pair aileron pulse, bank release, and rudder pulse with roll rate, spiral
bank/heading, and Dutch-roll sideslip/yaw; then explain how rate damping, oscillatory damping, and a
spiral-stability sign produce three different observable outcomes.
