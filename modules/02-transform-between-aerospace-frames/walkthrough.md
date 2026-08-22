# P02 walkthrough: Transform Between Aerospace Frames

## Guiding question

What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?

Follow one visual transition at a time. Do not open the interactive controls until after observing the fixed experiment.

1. Read the body, wind, and North-East-Down conventions in `lesson.md`.
2. Make one prediction: if only yaw changes, will speed change or only its North/East components?
3. Run the baseline section of `experiment.m`. The expected body velocity is approximately `[69.62, 0, 7.32] m/s`; the expected NED velocity is `[60.54, 34.95, -3.66] m/s`.
4. Inspect the body view, then the NED view. State why their coordinates differ while both vector norms remain `70 m/s`.
5. Run the yaw sweep. Observe track follow yaw while the `+3 deg` flight-path angle remains fixed, then read the first mechanism explanation.
6. Reset to the baseline and run the sideslip sweep. Observe `v = V sin(beta)` change sign and track move away from yaw, then read the second mechanism explanation.
7. Run the deliberately broken transpose case. A `+90 deg` yaw should send body-forward velocity east; the reversed transform points it west even though its norm remains `70 m/s`.
8. Open `interactive.m`. Move yaw alone, reset it, then move sideslip alone. Use the remaining controls only after explaining those two effects.
9. Run `run_module_checks("P02")` from the repository root.
10. Give the two-sentence teach-back from `checks.md`: mechanism first, failure diagnosis second.

The expected values are analytic reference values, not retained MATLAB-runtime evidence. This repository environment must execute the MATLAB checks separately before claiming runtime or UI validation.
