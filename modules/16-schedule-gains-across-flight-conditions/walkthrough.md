# P16 walkthrough: Schedule Gains Across Flight Conditions

## Guiding question

What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?

Follow one plot or processing transition at a time. Run the fixed experiment before opening
interactive controls.

1. Read the P15 connection in `lesson.md`. Explain why true airspeed is now a scheduling input and
   why P16 does not consume a P15 speed history.
2. State the units and signs: `V` in `m/s`, `rho` in `kg/m^3`, dynamic pressure in pascals, roll and
   equivalent aileron in degrees, roll rate in `deg/s`, and positive values right-wing-down.
3. Trace `qbar=0.5*rho*V^2`, normalized condition, plant effectiveness, gain lookup, equivalent
   aileron, roll acceleration, roll rate, and roll angle in that order.
4. Make one prediction: if dynamic pressure falls while both gains stay fixed, will the response
   remain equally damped?
5. Run the baseline section of `experiment.m`. Confirm `V=60 m/s`,
   `rho=0.736115547399152 kg/m^3`, `qbar/qbar_ref=1`, and the `0` to `10 deg` command at `0.5 s`.
6. Inspect only roll command and response. Observe about `1.23 s` to 90%, `1.55 s` settling, and
   `0.1615 deg` overshoot.
7. Inspect only the table. Follow the center knot into `K_phi=0.48 rad/rad` and `K_p=0.32 s`.
8. Inspect only equivalent aileron. Explain why it changes at command onset while roll angle and
   rate cannot jump.
9. Reset density and mode. Sweep true airspeed through `[45,52.5,60,67.5,72] m/s`.
10. Inspect the scheduled roll overlays before the metrics. Confirm settling remains within
    `1.55–1.56 s`.
11. Inspect peak equivalent aileron. Explain why rising `V^2` lets the declared plant use less
    control angle for nearly the same response.
12. Compare fixed-reference gains at the same speeds. At `45 m/s`, connect lower effective
    frequency and damping to about `3.29 s` settling and `0.982 deg` overshoot.
13. Reset airspeed to `60 m/s`. Sweep density through
    `[0.5,0.75,1,1.25,1.5]*rho_ref` with correct scheduling.
14. Inspect the overlaid roll histories. Use `b K_phi` and `b K_p` to explain why exact knot cases
    retain target poles.
15. Inspect peak equivalent aileron separately. State why an identical response can require
    different gains and command demand.
16. Compare scheduled and fixed modes at the reference knot. Confirm they are exactly equal and
    name this the reference limiting case.
17. Form the equal-pressure pair: `60 m/s` at `rho_ref`, then `75 m/s` at
    `rho_ref*(60/75)^2`. Confirm correct lookup produces equal histories.
18. Run the deliberately broken true-airspeed-only lookup at the second condition. Observe actual
    pressure ratio `1`, raw lookup `1.5625`, used lookup `1.5`, and an active clamp flag.
19. Follow only roll response. Diagnose the slower, less-damped trace from gains that are too small
    for the unchanged plant.
20. State why endpoint clamping bounded lookup but did not recover the correct schedule.
21. Open `interactive.m`. Move speed alone, reset; move density alone, reset; compare fixed gains;
    then use the broken source once and restore dynamic-pressure scheduling.
22. Run `run_module_checks("P16")` from the repository root, then give the two-sentence teach-back
    from `checks.md`.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, plots, UI callbacks, or MATLAB numerical
validation.
