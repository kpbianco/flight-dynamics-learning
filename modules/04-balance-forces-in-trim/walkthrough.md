# P04 walkthrough: Balance Forces in Trim

## Guiding question

What inputs, observable effects, and failure modes matter when you balance Forces in Trim?

Follow one visual transition at a time. Do not open the interactive controls until after observing
the fixed experiment.

1. Read the path-axis equations in `lesson.md`. Connect P03 density and true airspeed to dynamic pressure, then distinguish angle of attack from pitch attitude.
2. Make one prediction: at fixed density and mass, must required lift coefficient rise or fall when true airspeed decreases?
3. Run the baseline section of `experiment.m`. At the P03 5 km density, `60 m/s`, and `1200 kg`, expect approximately `q = 1325.01 Pa`, `CL = 0.5482`, `alpha = 3.42 deg`, and thrust `826.95 N`.
4. Inspect only the signed normal and along-path force bars. State what it means for each opposing pair to sum to zero.
5. Inspect the drag decomposition. Identify parasite and induced drag before adding them to recover required level-flight thrust.
6. Run the true-airspeed sweep. Observe `CL` and alpha fall with speed while parasite drag rises and induced drag falls. Then read the first mechanism explanation.
7. Reset to `60 m/s`, run the mass sweep, and observe parasite drag stay fixed while `CL`, alpha, induced drag, and thrust rise. Then read the second mechanism explanation.
8. Run the deliberately broken dynamic-pressure case. Missing the one-half commands half the needed lift and leaves a negative half-weight residual.
9. Open `interactive.m`. Move true airspeed alone, reset it, then move mass alone. Use density and flight-path angle only after explaining the required sweeps and the feasibility indicators.
10. Run `run_module_checks("P04")` from the repository root.
11. Give the two-sentence teach-back from `checks.md`: mechanism first, failure diagnosis second.

The expected values are analytic reference values, not retained MATLAB-runtime evidence. MATLAB must
execute the lesson separately before anyone claims runtime, UI, plot, or MATLAB numerical validation.
