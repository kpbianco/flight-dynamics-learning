# P11 checks: Model Flight Sensors

## Guiding question

What inputs, observable effects, and failure modes matter when you model Flight Sensors?

Ask and answer one item at a time.

## Observation check

On the baseline plots, point to true pitch rate, biased gyro rate, true pitch angle, accumulated
angle error, NED coordinate acceleration, ideal body specific force, and additive accelerometer
error. State each frame and unit. Why can a `0.20 deg/s` rate separation become `1.60 deg` of angle
error over eight seconds?

## P10 boundary check

Trace the curriculum chain from P10 delivered actuator authority through rigid-body truth to the
P11 measurement equations. Which interface is absent today, why can P10's output not be passed
directly to P11's two-scalar API, and which later module rather than P11 owns sensor fusion?

## First-lever check

Hold accelerometer error at `0.15 m/s^2` vector RMS and change gyro bias. Predict measured pitch
rate, final angle error, ideal/complete/broken accelerometer histories, and truth. Why does bias sign
set drift direction, and why does the error grow after the true maneuver has ended?

## Second-lever check

Reset gyro bias to `0.20 deg/s`, then change accelerometer-error vector RMS. Predict the additive
error amplitude, ideal specific force, gyro histories, and truth. Why does the observed vector RMS
equal the lever even though no random generator is used? Why must this not be called a measured
standard deviation or hardware-noise result?

## Frame, limiting-case, and interpretation checks

- Write `f_b=C_n_to_b*(a_n-g_n)` and identify the frame and units of every term.
- Why does a supported level sensor read `-g` on body z-down while its NED coordinate acceleration
  is zero? How is free fall different?
- Why is `C_n_to_b` the transpose of `C_b_to_n` for this orthonormal rotation?
- Show that `C_b_to_n*f_b+g_n=a_n` for the ideal output.
- Why does zero gyro bias reproduce both rate and integrated-angle truth?
- Why does zero accelerometer-error RMS reproduce ideal specific force exactly?
- Why must changing either sensor lever leave prescribed truth unchanged?
- Why is a constant gyro bias distinct from zero-mean sensor error?
- Which unmodeled effects prevent this deterministic teaching model from being sensor-fidelity
  evidence?

## Broken-case check

The deliberately broken calculation omits gravity while preserving truth, attitude, frame
transform, gyro, grid, and additive accelerometer error. Explain why zero ideal output at supported
level rest is the signature of confusing coordinate acceleration with specific force. Why does the
complete-minus-broken magnitude stay exactly `9.80665 m/s^2`, and why can a smooth finite trace
still corrupt a later estimator?

## Range, malformed-input, recovery, isolation, and resource check

`run_checks.m` accepts gyro bias only in `[-0.5,0.5] deg/s` and accelerometer-error vector RMS only
in `[0,0.5] m/s^2`. It rejects below/above-range, nonscalar, complex, `NaN`, and `Inf` inputs, then
repeats a valid call to prove recovery. Independent sweeps retain the non-lever sensor histories
exactly, checking isolation. The UI reset restores exact baseline values.

Each call is synchronous, stateless, and fixed at 801 samples and 800 updates. Corner and
representative-grid checks are explicitly capped. There is no file, network, device, process,
timer, future, or parallel operation to time out or cancel, so timeout and cancellation are not
runtime semantics of this model API. The learner CLI retains its separate ten-second subprocess
timeout in isolated fixtures.

Base MATLAB arithmetic and graphics are the intended compatibility boundary. MATLAB release,
graphics, callbacks, accessibility, Octave, Windows, and PowerShell behavior require execution in
those named environments. No learner data or schema migration occurs. Rollback removes only
P11-owned implementation artifacts and restores P11 lifecycle fields after coordinating any later
dependent frontier; no backup restore or service recovery action is required.

## Executable checks

`run_checks.m` covers determinism, fixed finite shapes, independent truth/frame/noise reconstruction,
every integration update, DCM and specific-force closure, a nonlevel frame-sign signature at the
peak-attitude/peak-acceleration sample, recognizable baseline values, exact rest and ideal limits,
two isolated sweeps, the gravity-omission symptom, malformed-input rejection, recovery,
compatibility declarations, and bounded work.

Run from the repository root:

```matlab
run_module_checks("P11")
```

## Teach-back

In two sentences, answer the guiding question. Sentence one must trace gyro bias into angle drift and
NED truth/gravity through the frame transform into body specific force, naming units and both
independent levers. Sentence two must diagnose the broken gravity-omission case from its zero-at-rest
symptom and exact `g` discrepancy without referring to MATLAB syntax.
