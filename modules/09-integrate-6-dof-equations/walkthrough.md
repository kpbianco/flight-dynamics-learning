# P09 walkthrough: Integrate 6-DOF Equations

## Guiding question

What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?

Follow one visual transition at a time. Do not open the interactive controls until after observing
the fixed experiment.

1. Read the P08 connection in `lesson.md`. Identify which P08 outputs could become P09 force and
   moment inputs, and which complete states P08 did not propagate.
2. Point body `x` forward, `y` right, `z` down; then point North, East, and Down. State in words that
   `C_body_to_ned` maps body components into NED.
3. Read the four governing equations. Locate the two cross products and explain why one transports
   velocity components while the other transports angular momentum components.
4. Make one prediction: after the positive three-axis moment pulse, can body velocity components
   change even when the inertial velocity vector does not change for that same reason?
5. Run the baseline section of `experiment.m`. Confirm 301 samples over `6 s`, then read only the
   final NED position `[328.492, 28.830, 13.362] m` and the peak body-rate magnitude
   `11.652 deg/s`.
6. Inspect only the horizontal NED path. Then inspect Down versus time and restate that positive Down
   means descent from the starting NED origin.
7. Inspect body velocity first, body rates second, and quaternion-derived Euler angles third. Do not
   treat any one of those coordinate histories as the complete motion.
8. Inspect the force ledger alone. Identify the finite `F_x` pulse and steady `F_z=-m g`. Then inspect
   the moment ledger and identify its shorter finite pulse.
9. Run the forward-force sweep with moment scale reset to `1`. Confirm increasing final North and
   speed while quaternion, rates, and applied moments remain exactly fixed; then read the first
   mechanism explanation.
10. Reset force scale to `1` and run the moment sweep. Confirm increasing peak body rate and attitude
    rotation, then observe how fixed body force maps into different East and Down paths; read the
    second mechanism explanation.
11. Evaluate the zero/zero and force-only limits in `lesson.md`. Explain why the former is exact and
    why the latter has an independent half-sine impulse calculation.
12. Run the deliberately broken transport case. Compare the complete and broken horizontal paths,
    then inspect only the complete-equation residual. Name the omitted `-omega cross velocity` term
    before reading the reported `132.792 m` separation.
13. Open `interactive.m`. Move forward-force scale alone and reset it. Then move moment scale alone;
    use the closure panel to distinguish a changed trajectory from a broken equation.
14. Run `run_module_checks("P09")` from the repository root.
15. Give the two-sentence teach-back from `checks.md`: the complete load-to-state chain first, then
    the omitted rotating-frame transport diagnosis.

The expected values are retained static and independent simulated-oracle references. MATLAB must
execute the lesson separately before anyone claims MATLAB runtime, Live Editor, figure, UI callback,
or MATLAB numerical validation.
