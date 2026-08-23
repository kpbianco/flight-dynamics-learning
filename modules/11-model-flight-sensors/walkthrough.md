# P11 walkthrough: Model Flight Sensors

## Guiding question

What inputs, observable effects, and failure modes matter when you model Flight Sensors?

Follow one plot or transition at a time. Run the fixed experiment before opening interactive
controls.

1. Read the P10 connection in `lesson.md`. Explain why delivered actuator authority must become
   rigid-body truth before a sensor can observe it, and why P10 and P11 are not directly composable.
2. State the frame and signs: body `x` forward, `y` right, `z` down; navigation NED; positive pitch
   nose-up; gravity `[0;0;9.80665] m/s^2` in NED.
3. Trace the two equations aloud: `q_measured=q_truth+bias` and
   `f_b=C_n_to_b*(a_n-g_n)+eta_b`.
4. Make one prediction: with a constant positive gyro bias, will final angle error return to zero
   when the true pitch-rate maneuver ends?
5. Run the baseline section of `experiment.m`. Confirm 801 samples over `8 s`, peak true pitch rate
   `10 deg/s`, peak pitch angle about `12.7321 deg`, and final true angle near zero.
6. Inspect only the gyro-rate view. Point out the small constant separation between truth and
   measurement before considering integration.
7. Inspect only the integrated-angle-error view. Explain the straight `bias*time` drift and the
   baseline `1.60 deg` final error.
8. Inspect only the ideal accelerometer view. At supported level rest, identify `-g` on body z-down;
   during the pulse, distinguish NED coordinate acceleration from body specific force.
9. Hold accelerometer error at `0.15 m/s^2`, run the gyro-bias sweep, and read final angle errors
   `[-4,-2,0,2,4] deg`. Confirm all accelerometer histories remain unchanged.
10. Read the gyro mechanism explanation: integrating a constant offset produces linear drift.
11. Reset gyro bias to `0.20 deg/s`, run the accelerometer-error sweep, and compare error traces
    before the vector-RMS summary. Confirm all truth and gyro histories remain unchanged.
12. Read the accelerometer-error mechanism explanation. Explain why rescaling one fixed normalized
    waveform changes magnitude but is not evidence of random or identified noise.
13. Inspect the zero-bias and zero-error limits. Name exactly which measured histories collapse onto
    their ideals.
14. Run the deliberately broken gravity-omission comparison. At supported level rest, distinguish
    the complete `-9.80665 m/s^2` body-z ideal reading from broken zero.
15. Confirm that complete-minus-broken measurement magnitude remains exactly `g`, then name the
    violated specific-force equation rather than diagnosing instability or noise.
16. Open `interactive.m`. Move gyro bias alone, reset, move accelerometer-error RMS alone, then reset
    again. Check the summary for exact lever isolation.
17. Run `run_module_checks("P11")` from the repository root.
18. Give the two-sentence teach-back from `checks.md`: correct equations and observables first, then
    the gravity-omission failure diagnosis.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, figure, UI callback, or MATLAB numerical
validation.
