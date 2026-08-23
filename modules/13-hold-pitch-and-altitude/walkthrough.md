# P13 walkthrough: Hold Pitch and Altitude

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?

Follow one plot or transition at a time. Run the fixed experiment before opening interactive
controls.

1. Read the P12 connection in `lesson.md`. State `h=-NED Down`, then form the correct altitude
   error `h_command-h`. Explain why P13 uses P12 conceptually rather than accepting a P12 history.
2. Trace the cascade without MATLAB syntax: altitude error → pitch command → pitch-control effect →
   pitch → flight-path angle → climb rate → altitude.
3. Treat `theta` and `gamma` as perturbations about level trim, then distinguish pitch from
   flight-path angle. Point to
   `gamma_dot=(theta-gamma)/tau_gamma` and `h_dot=V*sin(gamma)`.
4. Make one prediction: after an upward command, which moves first—pitch, flight-path angle, or
   altitude?
5. Run the baseline section of `experiment.m`. Confirm fixed `60 m/s` speed, a command from
   `1000 m` to `1030 m` at `1 s`, `K_h=0.004 rad/m`, and `omega_n=2.4 rad/s`.
6. Inspect only the altitude figure. Verify exact pre-step trim, then identify capture, overshoot,
   and final contraction of `h_command-h`.
7. Inspect only pitch command and pitch angle. At the command sample, pitch command is positive while
   pitch has not yet moved; explain why the controller must act before the state responds.
8. Add only the flight-path trace. At `1.5 s`, compare about `2.59069 deg` pitch with
   `0.292885 deg` path angle and explain the declared lift/path lag.
9. Inspect only equivalent pitch-control demand. Relate its positive sign to a nose-up effect and
   its magnitude to the inner-loop correction; do not reinterpret it as the physical-elevator sign
   declared in earlier modules.
10. Reset pitch frequency to `2.4 rad/s` and feedback sign to correct. Sweep altitude gain through
    `[0,0.002,0.004,0.006,0.008] rad/m`.
11. Start with `K_h=0`. Confirm the command changes while pitch, path angle, control, and altitude
    remain exactly at trim; name this the open-altitude-loop limit.
12. Inspect error at `5 s` across the gain sweep. Explain why more radians per metre reduces early
    error.
13. Inspect overshoot and pitch-command saturation. Explain why path lag and the `10 deg` envelope
    make gain a trade rather than free performance.
14. Reset `K_h=0.004 rad/m` and correct sign. Sweep inner natural frequency through
    `[1.2,1.8,2.4,3.0,3.6] rad/s`.
15. Inspect pitch at `1.5 s` and pitch-tracking RMS before looking at altitude. Explain how the
    scheduled gains make the inner loop react sooner while fixed damping remains declared.
16. Inspect peak pitch-control demand. State the trade: faster pitch tracking spends more
    control authority, while the unchanged path lag still separates `theta` from `gamma`.
17. Run the deliberately broken `s=-1` case. Confirm its state histories match baseline through the
    command sample while its command signals already have opposite signs there; then observe the
    negative pitch command drive the states away from the positive altitude command.
18. Follow one broken altitude-error trace. Diagnose positive feedback from the fact that the
    correction grows error in the same direction; do not blame windup, noise, or actuator lag that
    the model does not contain.
19. Open `interactive.m`. Move outer gain alone, reset; move pitch frequency alone, reset; then use
    the sign switch once and restore correct feedback.
20. Run `run_module_checks("P13")` from the repository root, then give the two-sentence teach-back
    from `checks.md`.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, plot, UI callback, or MATLAB numerical
validation.
