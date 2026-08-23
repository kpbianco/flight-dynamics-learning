# P07 walkthrough: Excite Roll, Spiral, and Dutch-Roll Modes

## Guiding question

What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?

Follow one visual transition at a time. Do not open the interactive controls until after observing
the fixed experiment.

1. Read the P06 connection in `lesson.md`. State how excitation, observables, time scale, and envelope identify a mode.
2. Read the sign conventions. Point body `y` right and explain why positive nose-right `r` initially makes `beta_dot` negative when flight path is frozen.
3. Make one prediction: which baseline signature disappears first—roll rate, Dutch-roll sideslip, or spiral bank?
4. Run the baseline section of `experiment.m`. Expect `tau_R = 0.4 s`, `T_D = 5.55436 s`, and `tau_S = 40 s`.
5. Inspect only roll rate. Then inspect its bank-angle integral and explain why rate damping does not command wings level.
6. Inspect only Dutch-roll sideslip and its envelope. Then inspect yaw rate and connect `beta_dot ~= -r` to their phase relationship.
7. Inspect only spiral bank over `120 s`. Then inspect heading and identify the small-angle coordinated-turn approximation.
8. Run the roll-decay-rate sweep. Confirm identical `p(0+)`, faster settling, less integrated bank, and unchanged spiral/Dutch-roll outputs; then read the first mechanism explanation.
9. Reset `lambda_R = 2.5 1/s`, run the Dutch-roll damping sweep, and inspect the amplitude retained per cycle. Connect its square to energy retained at the same phase; confirm roll/spiral isolation, then read the second explanation.
10. Run the deliberately broken spiral-sign case. Identify why the early response can look harmless, when it crosses `15 deg`, and why later values are symptoms rather than predictions.
11. Open `interactive.m`. Move the aileron pulse alone and reset; move bank release alone and reset; move rudder pulse alone and reset.
12. Set spiral decay to zero. Observe persistent bank and linear proxy heading without calling that a trimmed full-aircraft turn.
13. Move one shaping control at a time. Distinguish roll decay rate, stable-to-neutral spiral decay rate, and Dutch-roll damping. Diagnose the separate fixed broken-case plot to reason about spiral-sign reversal.
14. Run `run_module_checks("P07")` from the repository root.
15. Give the two-sentence teach-back from `checks.md`: excitation/observable pairing first, then failure-sign diagnosis.

The expected values are analytic reference values, not retained MATLAB-runtime evidence. MATLAB must
execute the lesson separately before anyone claims runtime, UI, plot, or MATLAB numerical validation.
