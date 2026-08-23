# P11 — Model Flight Sensors

**Track:** Flight Dynamics and Aerospace GNC

**Phase 3:** Six-degree-of-freedom simulation

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you model Flight Sensors?

## Physical mental model

P10 separated commanded actuator motion from delivered control authority. A later vehicle model
would propagate that delivered force and moment into truth states; P11 opens the next boundary and
turns prescribed truth into sensor measurements:

```text
pitch-rate truth q (deg/s) -> add constant gyro bias -> measured q -> integrate -> angle drift

attitude C_b_to_n + NED acceleration a_n + NED gravity g_n
                    -> f_b = C_n_to_b (a_n-g_n)
                    -> add body-frame teaching error eta_b -> accelerometer measurement
```

The body frame is `x` forward, `y` right, `z` down. The navigation frame is North-East-Down (NED),
and positive pitch is nose-up. An accelerometer measures *specific force*, not coordinate
acceleration. Consequently, a supported level sensor with zero coordinate acceleration reports
approximately `-9.80665 m/s^2` on body `z`, not zero.

This is a conceptual continuation from P10, not a package connection. P10 exposes a scalar
actuator/moment history on a different grid, while P11 prescribes truth internally and accepts only
two sensor-error levers. No hidden adapter, rigid-body propagation, or feedback loop is claimed.

## Deterministic experiment

The fixed `0:0.01:8 s` grid retains 801 samples and 800 trapezoidal updates. A sinusoidal pitch-rate
lobe runs from `1` to `5 s`, peaks at `10 deg/s`, and returns the vehicle to level after reaching
about `12.7321 deg`. A separate North-acceleration pulse runs from `2` to `4 s` and peaks at
`2 m/s^2`.

The baseline adds `0.20 deg/s` gyro bias and a deterministic, zero-mean, normalized body-frame
accelerometer error with `0.15 m/s^2` three-axis vector RMS. The gyro bias produces exactly
`bias*time`, hence `1.60 deg` final angle error over eight seconds. The multitone accelerometer
error is replayable teaching data; it is not white Gaussian noise, a random draw, a PSD, or
identified hardware noise.

These values are deterministic simulated equation references. They are not MATLAB-runtime,
graphics, UI, sensor-calibration, hardware, bench, HIL, or flight evidence.

## Two independent levers

1. Hold accelerometer error at `0.15 m/s^2` vector RMS and sweep gyro bias through
   `[-0.50,-0.25,0,0.25,0.50] deg/s`. Truth and all accelerometer histories remain fixed. Final
   integrated angle error follows `bias*8 s`: `[-4,-2,0,2,4] deg`.
2. Reset gyro bias to `0.20 deg/s` and sweep accelerometer error through
   `[0,0.05,0.15,0.30,0.50] m/s^2` vector RMS. Truth and all gyro histories remain fixed. The
   measured vector RMS of the additive error equals the requested magnitude.

Use the interactive reset button to restore exactly `0.20 deg/s` and `0.15 m/s^2` between levers.
The zero-bias limit reproduces gyro truth and integrated attitude. The zero-error limit reproduces
ideal specific force exactly.

## Deliberately broken gravity omission

The broken accelerometer calculation keeps the same time grid, attitude, coordinate acceleration,
gyro, and additive error but computes `C_n_to_b*a_n`, omitting only gravity. At supported level rest
its ideal reading is zero instead of `[0;0;-9.80665] m/s^2`. At every sample the magnitude between
complete and broken measurements is exactly `g`, because their shared additive error cancels.

The broken output stays finite and smooth. Its failure is semantic: it confuses coordinate
acceleration with accelerometer specific force. That plausible-looking zero-at-rest result would
give a later estimator the wrong measurement model.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P11")
run_module_checks("P11")
```

The implementation uses base MATLAB arithmetic, graphics, `uifigure`, and fixed synchronous loops.
It does not use a sensor toolbox, random generator, file or network I/O, device access, timers,
background processes, futures, or parallel workers. There is no asynchronous model task to time out
or cancel.

## Files

- `lesson.m` — sectioned entry point and concise truth-to-measurement narrative.
- `model.m` — guarded deterministic truth, frame transform, sensor errors, and broken comparison.
- `experiment.m` — baseline views, independent sweeps, limiting cases, and gravity omission.
- `interactive.m` — gyro-bias/noise controls, exact reset, and immediate measurement views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, mechanisms, and observation order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and an independent Python oracle can validate structure and simulated
reference results without MATLAB. They do not establish MATLAB syntax execution, MATLAB numerical
behavior, Live Editor order, plots, callbacks, learner understanding, sensor fidelity, hardware,
bench, HIL, field, release, deployment, or production evidence.
