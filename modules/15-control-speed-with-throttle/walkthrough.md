# P15 walkthrough: Control Speed with Throttle

## Guiding question

What inputs, observable effects, and failure modes matter when you control Speed with Throttle?

Follow one plot or processing transition at a time. Run the fixed experiment before opening
interactive controls.

1. Read the P14 connection in `lesson.md`. Explain why P14's fixed `60 m/s` condition motivates
   P15, then explain why P15 does not accept a P14 history or run heading hold.
2. State the units and signs: forward speed is positive in `m/s`, forward force is positive in
   newtons, throttle is a fraction in `[0,1]`, and acceleration is in `m/s^2`.
3. Trace parasite and induced drag separately. Identify the `V^2` and `1/V^2` mechanisms without
   treating this fixed-lift relation as a post-stall model.
4. Make one prediction: after the positive speed-command step, does acceleration move with the
   throttle request or only after delivered throttle moves?
5. Run the baseline section of `experiment.m`. Confirm `K_V=0.15 1/s`, `tau_T=0.8 s`, correct
   sign, and the `60` to `70 m/s` command at `1 s`.
6. Inspect only the speed plot. Observe exact pre-step trim, then follow speed monotonically toward
   command.
7. At the command sample, compare `+10 m/s` error, `+1.5 m/s^2` requested acceleration, and about
   `2626.952 N` commanded thrust with still-zero actual acceleration.
8. Inspect only requested and delivered throttle. Explain why requested throttle jumps to about
   `65.674%` while delivered throttle remains about `20.674%` at that sample.
9. Add only thrust and drag. Watch delivered thrust rise above drag before interpreting the
   positive acceleration plot.
10. Reset throttle time constant to `0.8 s` and feedback sign to correct. Sweep speed gain through
    `[0,0.075,0.15,0.225,0.3] 1/s`.
11. Begin at `K_V=0`. Confirm command changes while throttle, thrust, drag, acceleration, and speed
    remain at exact trim; name this the feedback-open limit.
12. Inspect speed at `5 s` and error at `10 s`. Explain why more desired acceleration per unit
    error contracts the early error.
13. Inspect peak delivered throttle and thrust-command saturation. Explain why the `4000 N`
    envelope makes gain an authority trade rather than free capture speed.
14. Reset `K_V=0.15 1/s` and correct sign. Sweep throttle time constant through
    `[0.2,0.5,0.8,1.1,1.4] s`.
15. Inspect delivered throttle and speed at `2 s` before looking at an endpoint. Explain why a
    smaller time constant moves the actuator and speed sooner.
16. Inspect throttle tracking RMS and peak throttle rate. State the trade: less lag costs more
    normalized throttle rate without proving engine feasibility.
17. Run the deliberately broken reversed-sign case. Confirm state histories match correct mode
    through command onset while commands become about `2626.952 N` versus idle.
18. Follow only true airspeed and proper command-minus-speed error. Diagnose positive feedback:
    falling speed makes the proper error grow while the broken controller continues asking for
    less thrust.
19. Inspect only the final retained second. Confirm idle command, continued deceleration, growing
    error, and an above-stall endpoint; do not extrapolate this to an infinite-horizon or post-stall
    claim.
20. Open `interactive.m`. Move gain alone, reset; move throttle time constant alone, reset; then
    use reversed feedback once and restore correct feedback.
21. Run `run_module_checks("P15")` from the repository root, then give the two-sentence teach-back
    from `checks.md`.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, plot, UI callback, or MATLAB numerical
validation.
