# P12 walkthrough: Validate Energy and Frame Conventions

## Guiding question

What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?

Follow one plot or transition at a time. Run the fixed experiment before opening interactive
controls.

1. Read the P11 connection in `lesson.md`. Rearrange
   `f_b=C_n_to_b*(a_n-g_n)` into `a_n=C_body_to_ned*f_b+g_n`, and explain why
   the connection is conceptual rather than a current API adapter.
2. State every sign before calculating: body `x` forward, `y` right, `z` down; navigation NED;
   positive pitch nose-up; heading clockwise from North toward East; altitude `h=-Down`.
3. Trace the work-energy audit aloud:
   `T=0.5*m*(v dot v)`, `U=-m*g*Down`, and
   `E-E0=integral(m*f dot v dt)`.
4. Make one prediction: during the climb, does Down increase or decrease, and should potential
   energy rise or fall?
5. Run the baseline section of `experiment.m`. Confirm 301 samples over `6 s`, fixed pitch
   `30 deg`, initial speed `60 m/s`, body-x specific force `1.5 m/s^2`, and heading `30 deg`.
6. Inspect only the NED trajectory. Point out positive North/East motion, the initial climb, and the
   use of altitude rather than Down on the vertical axis.
7. Inspect only the NED velocity components. Confirm initial `[45;25.9808;-30] m/s`; explain why
   nose-up body-forward motion has negative Down velocity.
8. Inspect only mechanical-energy change and accumulated non-gravity work. Confirm that both reach
   about `537.732 kJ` rather than expecting total energy to remain constant with force applied.
9. Inspect the three closure traces. Distinguish energy-minus-work `[J]`, body-minus-NED kinetic
   energy `[J]`, and body-minus-NED power `[W]`.
10. Hold heading at `30 deg`, sweep body-x specific force, and examine altitude histories before
    the summary curve. Confirm the DCM and initial state remain fixed.
11. Inspect work and apex gain across `[0,0.75,1.5,2.25,3] m/s^2`. Explain why positive work and
    the force's negative-Down component raise both final energy and apex.
12. Inspect the `f_x=0` limit. State why ideal accelerometer output, non-gravity power, and work are
    zero in free fall while coordinate acceleration is still gravity.
13. Reset `f_x=1.5 m/s^2`, sweep heading, and view one horizontal path at a time. Confirm heading
    `+90 deg` points East and `-90 deg` points West.
14. Read the heading mechanism explanation. Identify the lever as an active yaw of the body and path
    within fixed NED, not a passive relabeling. Verify that uniform gravity and no horizontal
    asymmetry leave body velocity, Down/altitude, speed, power, work, energy, and apex unchanged
    while North/East histories rotate.
15. Run the deliberately broken Down-as-height comparison. At the initial datum, notice that both
    residuals are zero; explain why one-point validation is insufficient.
16. Follow the whole broken trace and compare it with
    `2*m*g*(Down-Down0)`. Diagnose the potential-energy sign rather than drag, instability, sensor
    noise, or integration error.
17. Open `interactive.m`. Move specific force alone, reset, move heading alone, then reset again.
    Check the summary for frame, power, work, and energy isolation.
18. Run `run_module_checks("P12")` from the repository root, then give the two-sentence teach-back
    from `checks.md`.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, plot, UI callback, or MATLAB numerical
validation.
