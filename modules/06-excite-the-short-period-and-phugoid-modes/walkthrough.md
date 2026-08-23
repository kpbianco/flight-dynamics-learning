# P06 walkthrough: Excite the Short-Period and Phugoid Modes

## Guiding question

What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?

Follow one visual transition at a time. Do not open the interactive controls until after observing
the fixed experiment.

1. Read the P05 connection in `lesson.md`. State why negative `C_m_alpha` proves an initial restoring tendency but does not prove damping.
2. Make one prediction: after the baseline trailing-edge-up elevator pulse, will fast angle of attack or slow speed/altitude exchange become visible first?
3. Run the baseline section of `experiment.m`. Expect `omega_sp = 4.50066 rad/s`, `T_sp = 1.49032 s`, `omega_ph = 0.231145 rad/s`, and `T_ph = 27.2703 s`.
4. Inspect only the fast `delta alpha` plot. Identify its decaying envelope before inspecting pitch rate.
5. Inspect pitch rate. Explain why the pulse sets `q(0+)` while `delta alpha(0) = 0`, and why `alpha_dot ~= q` requires nearly frozen flight path.
6. Inspect only the slow `delta V` plot. Then inspect `gamma`, followed by altitude, and describe the speed-to-height exchange without calling `gamma` pitch attitude.
7. Run the short-period damping sweep. Observe only the fast response and decay-per-period metric; confirm the phugoid stays identical, then read the first mechanism explanation.
8. Reset `zeta_sp = 0.35`, run the phugoid damping sweep, and observe the zero-damping limit before the positive-damping traces; confirm the short-period response stays identical, then read the second mechanism explanation.
9. Run the deliberately broken damping-sign case. Identify why unchanged restoring stiffness and frequency do not prevent growth, and why the trace beyond `+/-5 deg` is a failure symptom rather than a physical prediction.
10. Open `interactive.m`. Move the elevator pulse alone and then reset it; move the airspeed kick alone and then reset it. Name which observables disappear at each zero-input limit.
11. Move one damping slider at a time. Separate amplitude of excitation from rate of decay.
12. Run `run_module_checks("P06")` from the repository root.
13. Give the two-sentence teach-back from `checks.md`: excitation/observable pairing first, damping-sign diagnosis second.

The expected values are analytic reference values, not retained MATLAB-runtime evidence. MATLAB must
execute the lesson separately before anyone claims runtime, UI, plot, or MATLAB numerical validation.
