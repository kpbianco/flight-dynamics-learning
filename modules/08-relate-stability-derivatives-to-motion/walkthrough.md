# P08 walkthrough: Relate Stability Derivatives to Motion

## Guiding question

What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?

Follow one visual transition at a time. Do not open the interactive controls until after observing
the fixed experiment.

1. Read the P07 connection in `lesson.md`. Explain the difference between prescribing a modal shape and deriving coupled motion from derivative contributions.
2. Point body `y` right and restate the signs of `beta`, `p`, `r`, and `phi`.
3. Read the chain from state through normalized rate, coefficient, load, acceleration, and next state. Identify where `b/(2V0)` enters.
4. Make one prediction: for a positive sideslip release, what initial signs should negative `C_l_beta` and positive `C_n_beta` create in `p_dot` and `r_dot`?
5. Run the baseline section of `experiment.m`. Expect `p_dot(0) = -33.691667 deg/s^2`, `r_dot(0) = +31.585938 deg/s^2`, and the first beta zero at `0.54 s`.
6. Inspect only sideslip, then yaw rate. Connect the `C_n_beta beta` yaw moment and the `-r` kinematic term.
7. Inspect only roll rate, then bank angle. Connect the `C_l_beta beta` moment, developed `C_l_p p_hat` damping, and `phi_dot=p`.
8. Inspect the roll ledger one contribution at a time. Then inspect the yaw ledger; explain why the totals change as states feed back.
9. Run the `C_l_p` sweep. Confirm the same initial roll acceleration, smaller developed roll-rate and bank peaks, and exactly one changed matrix entry; then read the first mechanism explanation.
10. Reset `C_l_p=-0.50` and run the `C_n_beta` sweep. Confirm linear initial yaw acceleration and earlier beta zero crossings; then explain why `C_n_beta=0` does not suppress all later yaw.
11. Run the deliberately broken rate-normalization case. Identify the missing `b/(2V0)`, compare the correct `-4.25045 1/s` entry with the broken `-46.79398` numeric value carrying `1/s^2`, and explain why their `11.00917 1/s` numeric-value quotient is not a dimensionless multiplier or evidence of correct units.
12. Open `interactive.m`. Move initial sideslip alone and reset; verify zero input and sign symmetry.
13. Move `C_l_p` alone and reset. Then move `C_n_beta` alone. Follow direct ledger changes before interpreting all coupled state changes.
14. Run `run_module_checks("P08")` from the repository root.
15. Give the two-sentence teach-back from `checks.md`: the full derivative-to-motion chain first, then the normalization failure diagnosis.

The expected values are independent analytic and fixed-step reference results, not retained MATLAB-
runtime evidence. MATLAB must execute the lesson separately before anyone claims runtime, UI, plot,
callback, or MATLAB numerical validation.
