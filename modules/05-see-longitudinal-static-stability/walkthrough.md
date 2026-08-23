# P05 walkthrough: See Longitudinal Static Stability

## Guiding question

What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?

Follow one visual transition at a time. Do not open the interactive controls until after observing
the fixed experiment.

1. Read the station and moment signs in `lesson.md`. Connect P04's `q` and required alpha to P05's locally retrimmed incremental reference.
2. Make one prediction: after positive `delta alpha`, must a statically stable aircraft initially create nose-up or nose-down moment?
3. Run the baseline section of `experiment.m`. Expect `h_n = 50.8468% MAC`, static margin `20.8468% MAC`, `C_m_alpha = -1.1324 /rad`, and `delta M = -1272.72 N*m` for `+2 deg`.
4. Inspect only the moment-coefficient line. State what its negative slope means before inspecting the component bars.
5. Inspect the wing, tail, and total derivative bars. Add the wing and tail contributions and recover the total slope.
6. Run the CG sweep. Observe the neutral point stay fixed while static margin and restoring moment fall as CG moves aft; then read the first mechanism explanation.
7. Reset CG to `30% MAC`, run the tail-area sweep, and observe the neutral point move aft while the `+2 deg` moment becomes more nose-down; then read the second mechanism explanation.
8. Run the deliberately broken sign case. Identify why the same positive angle-of-attack disturbance now creates a nose-up reinforcing moment.
9. Open `interactive.m`. Move CG alone, reset it, then move tail area alone. Use angle-of-attack and elevator perturbations only after explaining the two required sweeps.
10. Move elevator with the geometry fixed. Observe the moment line shift while its slope and static margin remain unchanged.
11. Run `run_module_checks("P05")` from the repository root.
12. Give the two-sentence teach-back from `checks.md`: mechanism first, failure diagnosis second.

The expected values are analytic reference values, not retained MATLAB-runtime evidence. MATLAB must
execute the lesson separately before anyone claims runtime, UI, plot, or MATLAB numerical validation.
