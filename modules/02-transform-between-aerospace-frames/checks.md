# P02 checks: Transform Between Aerospace Frames

## Guiding question

What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?

Ask and answer one item at a time.

## Observation check

When yaw alone increases, which NED components change, which metrics remain fixed, and why?

## Second-lever check

After resetting yaw, make sideslip positive. Predict the sign of body lateral velocity before looking at the plot, then explain why track can differ from yaw.

## Limiting-case checks

- With all angles zero, where must a body-forward velocity point in NED?
- With only yaw at `+90 deg`, which NED component must be positive?
- With only positive pitch, why is the Down component negative?
- With positive sideslip and a `+90 deg` roll, why does body-right velocity become positive Down?
- Why must a proper direction cosine matrix preserve norm and have determinant `+1`?

## Broken-case check

The deliberately broken case uses the transpose of the body-to-NED matrix in the forward direction. Explain why the output points the wrong way even though its magnitude, matrix orthogonality, and determinant all look valid. “The matrix is wrong” is not a complete diagnosis.

## Transfer check

P01 plotted a North/East flight path. Explain what additional information P02 needs to turn a body-axis air-data velocity into that navigation-frame direction, and name the extra step required before air-relative velocity becomes ground velocity.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P02")
```

`run_checks.m` covers determinism, analytic sign and limiting cases, two sweep regressions, round-trip and proper-rotation invariants, malformed inputs, fixed-size outputs, and the norm-preserving transpose failure. All assertions must pass before learner completion.

## Teach-back

In two sentences: first explain how wind angles and attitude map one physical velocity through body and NED coordinates; then explain how a known-sign case catches a transform-direction error that a norm-only check misses.
