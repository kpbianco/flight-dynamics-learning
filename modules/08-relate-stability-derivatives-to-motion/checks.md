# P08 checks: Relate Stability Derivatives to Motion

## Guiding question

What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?

Ask and answer one item at a time.

## Observation check

For the baseline positive sideslip release, explain why `beta_dot` and `p_dot` begin negative while
`r_dot` begins positive. Name the derivative, kinematic, or gravity term responsible for each sign;
do not refer to plot color.

## Derivative-chain check

Trace `C_l_p` from dimensional roll rate through `p_hat = p b/(2V0)`, roll-moment coefficient,
dimensional moment `L = qbar S b C_l`, acceleration `p_dot = L/I_x`, and the next state. Repeat for
`C_n_beta` beginning with sideslip in radians. Which parts are coefficient slopes, which are reference-
condition scaling, and which are equations of motion?

## First-lever check

At fixed sideslip release and `C_n_beta`, make `C_l_p` more negative. Predict initial roll
acceleration, peak roll rate, peak bank, and the directly changed state-matrix entry. Explain why
coupled beta and yaw histories may also change even though the sweep changes one derivative only.

## Second-lever check

Reset `C_l_p=-0.50`, then increase `C_n_beta`. Predict initial yaw acceleration, first sideslip zero
crossing, peak yaw rate, and the directly changed matrix entry. At `C_n_beta=0`, explain why initial
yaw acceleration is zero but later yaw rate is not.

## Limiting-case and interpretation checks

- Why does zero initial sideslip with zero initial `p`, `r`, and `phi` produce exact zero histories?
- Why does reversing initial sideslip reverse every state, force, and moment but preserve peak magnitudes?
- Why does halving the release halve every linear history?
- Why are `p_hat` and `r_hat` dimensionless even though `b/(2V0)` has units of seconds?
- Why does `C_l_p` contribute nothing to `p_dot(0)` when `p(0)=0`?
- Why does positive `C_n_beta` describe restoring weathercock stability rather than damping?
- Why are `-r` and `g phi/V0` not stability derivatives?
- Why does one changed derivative alter several histories in a coupled state model?
- Why must coefficient contributions sum before dimensional loads divide by mass or inertia?
- Why are bounded early peaks insufficient to rule out a slow divergent mode, and what does contraction
  from the `15-20 s` state envelopes to the `20-25 s` envelopes establish without proving stability
  outside the sampled lesson horizon?
- Why does the visible RK4 recurrence not make P08 a complete six-degree-of-freedom integrator?

## Broken-case check

The deliberately broken case omits `b/(2V0)` from `C_l_p`, as if the nondimensional derivative
multiplied dimensional `p` directly. Compare the correct `-4.25045 1/s` matrix entry with the broken
numeric value `-46.79398`, whose physical units are instead `1/s^2`. Explain why their SI
numeric-value quotient is `11.00917 1/s`, not a dimensionless multiplier; why initial roll
acceleration is unchanged; and why the smooth reductions to `0.647583 deg/s` peak roll rate and
`0.184170 deg` peak bank are warning symptoms rather than improved fidelity. Then explain the
separate `180/pi` error caused by feeding degree numbers to per-radian derivatives.

## Range, resource, and transfer check

Explain why the model bounds initial sideslip, `C_l_p`, and `C_n_beta`; uses a fixed 1,251-sample
grid and synchronous recurrence; rejects malformed inputs before propagation; and makes no timeout
or cancellation promise because no background operation exists. Identify what P09 must add before
nonlinear six-degree-of-freedom motion is present.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P08")
```

`run_checks.m` covers determinism, dimensional derivative reconstruction, all four state equations,
the complete RK4 recurrence, coefficient/load closure, fixed finite resources, eight accepted corners,
a capped 27-case grid, contracting late-window motion at the accepted derivative corners, sign and scale
symmetry, zero release, zero weathercock derivative, both isolated sweeps, malformed inputs, recovery,
degree/radian misuse, and the broken rate normalization. All assertions must pass before learner
completion.

## Teach-back

In two sentences: first trace sideslip and body rates through normalized derivative contributions,
dimensional loads, accelerations, and coupled motion; then explain how omitting `b/(2V0)` can produce a
smooth answer whose units and motion are still wrong.
