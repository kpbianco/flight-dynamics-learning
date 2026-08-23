# P10 walkthrough: Model Actuator Dynamics and Limits

## Guiding question

What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?

Follow one plot or signal transition at a time. Run the fixed experiment before opening the
interactive controls.

1. Read the P09 connection in `lesson.md`. Identify P09's internal applied body-y moment and state
   why the conceptual match is not a directly composable public API.
2. Trace the signal chain aloud: requested command, position-limited command, lag rate demand,
   rate-limited motion, delivered deflection, and delivered pitch moment.
3. State the units before plotting: deflection in `deg`, rate in `deg/s`, time constant and time in
   `s`, and pitch moment in `N*m`.
4. Make one prediction: immediately after the command reverses from `+25 deg` to `-25 deg`, will the
   time constant or the `45 deg/s` rate stop set the delivered slope?
5. Run the baseline section of `experiment.m`. Confirm 501 samples over `5 s`, a `+/-15 deg` hard
   stop, and separate requested, feasible, and delivered signals.
6. Inspect only the command/deflection view. At `0.5 s`, distinguish the infeasible `+25 deg`
   request, the feasible `+15 deg` target, and the moving surface.
7. Inspect only the rate view. Compare the raw `83.333 deg/s` initial demand with the delivered
   `45 deg/s`, then watch the lag regain control as position error shrinks.
8. Inspect only the moment view. Explain which delayed delivered moment a later rigid-body adapter
   would use, rather than either requested curve.
9. Reset the rate limit to `45 deg/s` and run the time-constant sweep. Observe deflection traces
   first, then read the separate time-to-90%-of-feasible-target and feasible-RMS-error axes.
10. Read the first mechanism explanation. Explain why larger `tau` lowers raw demand for the same
    remaining error without guaranteeing pointwise raw-rate ordering across different trajectories.
11. Reset `tau` to `0.18 s` and run the rate-limit sweep. Observe the reversal slopes first, then
    compare the three observables on their separate unit-specific axes.
12. Read the second mechanism explanation. Name the transition from a straight rate-limited segment
    to a curved lag-governed segment.
13. Inspect the `tau=0.50 s`, `120 deg/s` limiting case. Verify that no sample reaches the rate stop
    and trace one update through the explicit first-order recurrence. Explain why the post-update
    state clip is a defensive guard that remains inactive on the accepted domain.
14. Run the deliberately broken position-envelope comparison. Check deflection against `+/-15 deg`
    before looking at moment; identify the roughly `9.987 deg` position violation and distinguish
    the `1998.952 N*m` broken peak from its `798.952 N*m` infeasible excess.
15. Open `interactive.m`. Move time constant alone, then use the reset button. Move rate authority
    alone, reset again, and use the broken panel to keep smoothness separate from envelope compliance.
16. Run `run_module_checks("P10")` from the repository root.
17. Give the two-sentence teach-back from `checks.md`: the complete command-to-moment chain first,
    then the omitted-position-envelope diagnosis.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, figure, UI callback, or MATLAB numerical
validation.
