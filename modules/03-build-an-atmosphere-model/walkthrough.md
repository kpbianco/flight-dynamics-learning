# P03 walkthrough: Build an Atmosphere Model

## Guiding question

What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?

Follow one visual transition at a time. Do not open the interactive controls until after observing the fixed experiment.

1. Read the physical chain in `lesson.md`. Connect P02's air-relative vector magnitude to P03 true airspeed, and state why positive-up altitude is not NED Down.
2. Make one prediction: at fixed `150 m/s` true airspeed, will climbing increase or decrease dynamic pressure?
3. Run the baseline section of `experiment.m`. At `5000 m` and `0 K` offset, expect approximately `255.65 K`, `54.02 kPa`, `0.7361 kg/m^3`, `Mach 0.468`, and `q = 8.28 kPa`.
4. Inspect the normalized atmospheric state, then the local dynamic-pressure curve. Explain why true airspeed alone cannot determine `q`.
5. Run the altitude sweep. Observe pressure and density fall, temperature change equation at 11 km, and fixed-speed dynamic pressure decline. Then read the first mechanism explanation.
6. Reset to `5000 m` and run the temperature-offset sweep. Observe pressure stay fixed while warmer air lowers density and `q`, raises sound speed, and lowers Mach. Then read the second mechanism explanation.
7. Run the deliberately broken constant-density case. At 11 km, sea-level density predicts more than three times the correct dynamic pressure even though true airspeed is unchanged.
8. Open `interactive.m`. Move altitude alone, reset it, then move temperature offset alone. Move true airspeed only after explaining the two atmosphere effects.
9. Run `run_module_checks("P03")` from the repository root.
10. Give the two-sentence teach-back from `checks.md`: mechanism first, failure diagnosis second.

The expected values are analytic reference values, not retained MATLAB-runtime evidence. MATLAB must execute the lesson separately before anyone claims runtime, UI, plot, or MATLAB numerical validation.
