# P17 walkthrough: Fuse INS and GPS

## Guiding question

What inputs, observable effects, and failure modes matter when you fuse INS and GPS?

Follow one plot or processing transition at a time. Run the fixed experiment before opening the
interactive controls.

1. Read the P16 prerequisite connection in `lesson.md`. Explain why P17 needs state estimates but
   does not consume P16 controller histories.
2. State the scope and units: one-dimensional North, position in meters, velocity in `m/s`,
   acceleration in `m/s^2`, INS updates at `0.02 s`, and GPS fixes every `1 s`.
3. Trace `a_INS=a_truth+b_a`, position/velocity prediction, GPS innovation, gate decision, and
   alpha-beta correction in that order.
4. Make one prediction: with positive constant acceleration bias and no GPS corrections, does
   position error grow linearly or quadratically?
5. Run the baseline section of `experiment.m`. Inspect only truth and INS-only position first.
   Confirm the final dead-reckoned errors are `72 m` and `2.4 m/s`.
6. Add only the gated fused position. Observe about `1.1809 m` position RMS and explain why
   one-Hz absolute corrections bound accumulated bias without changing the sensor bias itself.
7. Inspect the GPS markers. Distinguish the fixed nominal error waveform from the one contaminated
   fix at `30 s`; neither is random hardware data.
8. Inspect only predicted-versus-corrected error. Confirm prediction exists before every GPS
   innovation and accepted correction.
9. Inspect only innovations and the `+/-25 m` gate. Confirm 59 fixes are accepted and one is
   rejected, with exactly zero correction at the rejected sample.
10. Reset GPS RMS to `1 m` and mode to gated. Sweep INS bias through
    `[0,0.02,0.04,0.06,0.08] m/s^2`.
11. Follow the INS-only final-error metric. Use `0.5*b_a*t^2` and `b_a*t` to explain the exact
    `[0,36,72,108,144] m` and `[0,1.2,2.4,3.6,4.8] m/s` sequences.
12. Inspect fused error separately. Explain why fixes bound drift while larger bias increases what
    each prediction asks the next correction to remove.
13. Reset bias to `0.04 m/s^2`. Sweep GPS position-error RMS through `[0,0.5,1,2,4] m`.
14. Confirm truth and INS-only histories stay identical. Observe that the nominal GPS waveform has
    exactly the selected RMS and fused error rises as accepted measurement error enters the state.
15. Run `model(0,0,1)`. Confirm the ideal sensor limit follows truth exactly while still rejecting
    the fixed outlier.
16. Select INS-only mode. Confirm all 60 fixes are classified as ignored rather than gate-rejected,
    and the selected fused arrays equal dead reckoning.
17. Restore baseline inputs and gated mode. Record the outlier innovation and zero correction.
18. Select the deliberately broken accept-all mode. Inspect the `30–31 s` transition only: the
    same outlier now creates a large positive position and velocity correction.
19. Follow the broken position error until the next fix. Explain why accepting the outlier creates
    additional growth and why later reconvergence does not make the failure harmless.
20. Use the exact reset. Move bias alone, reset; move GPS error alone, reset; then restore gated
    fusion after the two comparison modes.
21. Run `run_module_checks("P17")` from the repository root, answer one interpretation question at
    a time, and give the two-sentence teach-back from `checks.md`.

The retained numbers are static and independent simulated-oracle references. MATLAB must run the
lesson separately before anyone claims MATLAB runtime, plots, UI callbacks, or MATLAB numerical
validation.
