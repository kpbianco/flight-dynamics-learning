# P05 checks: See Longitudinal Static Stability

## Guiding question

What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?

Ask and answer one item at a time.

## Observation check

With aft-positive station and nose-up-positive moment conventions, why does a negative
`C_m_alpha` make the initial response to positive `delta alpha` restoring? Name the sign of the
moment rather than saying only that the line slopes down.

## First-lever check

At fixed tail area, move CG aft. Predict the direction of static margin, `C_m_alpha`, neutral-point
position, and the moment after `+2 deg delta alpha`. Explain which quantity stays fixed and why.

## Second-lever check

Reset CG to `30% MAC`, then increase horizontal-tail area alone. Predict the direction of tail
lift-curve contribution, neutral point, static margin, and restoring moment. Explain why the tail's
aft station matters as much as its lift response.

## Limiting-case checks

- At `h_cg = h_n`, why is `C_m_alpha = 0`, and why does that say nothing by itself about absolute trim?
- With no horizontal tail, why does `h_n` reduce to the wing aerodynamic center and elevator effectiveness vanish?
- Why are equal positive and negative angle-of-attack perturbations associated with equal and opposite alpha moments in this linear model?
- Why does `alpha_abs [deg] = alpha_ref,P04 [deg] + delta alpha [deg]` leave the P04 reference fixed while the disturbed absolute angle changes?
- Why does moving CG change the slope while an elevator perturbation changes the intercept but not the stick-fixed slope?
- Why must `delta M = q S c_bar delta C_m` include the mean aerodynamic chord to have moment rather than force units?

## Broken-case check

The deliberately broken case defines static margin as `h_cg - h_n` and still uses a leading minus
sign. Explain why this turns a forward-CG configuration's response to positive `delta alpha` from
nose-down to nose-up. “The sign is wrong” is incomplete: name the reversed distance, the moment
convention, and the reinforcing observable.

## Range and transfer check

Explain why every CG/tail geometry is treated as a separately retrimmed incremental reference, and
why zero `delta C_m` at zero perturbation does not predict the elevator change required after moving
CG or resizing the tail. Then distinguish static stability from the damping and time histories that
P06 will add.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P05")
```

`run_checks.m` covers determinism, finite scalar resource bounds, independent component and
neutral-point equations, stable/neutral/unstable limits, alpha symmetry, elevator independence,
both experiment sweeps, malformed inputs, recovery, and the broken static-margin sign. All
assertions must pass before learner completion.

## Teach-back

In two sentences: first connect P04's reference air state, CG, and tail area to neutral point, static
margin, `C_m_alpha`, and the initial restoring moment; then explain how a reversed static-margin
subtraction reveals itself after a positive angle-of-attack disturbance.
