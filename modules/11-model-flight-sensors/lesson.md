# P11 lesson: Model Flight Sensors

## Guiding question

What inputs, observable effects, and failure modes matter when you model Flight Sensors?

## Compounds on P10

P10 turned a deflection request into delivered actuator motion and a conceptual body-y moment
ledger. A complete simulation would pass delivered forces and moments through rigid-body dynamics
before sensors observe the resulting truth. P11 focuses on that observation boundary: prescribed
pitch attitude, pitch rate, and NED acceleration become gyro and accelerometer measurements.

```text
P10 delivered authority -> rigid-body truth -> P11 measurement model -> later estimator/controller
```

The arrows describe curriculum order, not current API compatibility. P10 has a different grid and
does not publish a general rigid-body state adapter; P11 constructs its own truth schedule and takes
only gyro-bias and accelerometer-error inputs. This lesson adds neither the missing propagation
adapter nor P17 sensor fusion, P22 bus timing, or P23 fault-management behavior.

## Frames, truth, and measurements

The body frame uses `x` forward, `y` right, and `z` down. The navigation frame is
North-East-Down (NED), and positive pitch is nose-up. `C_b_to_n` maps body components into NED;
its transpose `C_n_to_b` maps NED components into body axes.

The fixed truth contains a `10 deg/s` sinusoidal pitch-rate lobe from `1` to `5 s` and a separate
`2 m/s^2` North-acceleration pulse from `2` to `4 s`. Trapezoidal integration on the explicit
`0.01 s` grid produces pitch attitude. Both sensor levers leave all of those truth histories fixed.

The pitch gyro equation is

```text
q_measured(t) = q_truth(t) + b_q
theta_measured(t) = integral q_measured(t) dt
```

A constant rate bias therefore becomes a linearly growing angle error:
`theta_error(t)=b_q*t`. The baseline `0.20 deg/s` bias is small in the rate view yet accumulates to
`1.60 deg` over eight seconds. A zero bias reproduces both rate and integrated-angle truth.

The ideal accelerometer equation is

```text
f_b(t) = C_n_to_b(t) [a_n(t)-g_n]
g_n    = [0; 0; 9.80665] m/s^2
f_measured_b(t) = f_b(t) + eta_b(t)
```

`f_b` is specific force. A supported level accelerometer has `a_n=0` but its support force prevents
free fall, so the ideal reading is `[0;0;-9.80665] m/s^2` in this z-down body convention. A freely
falling ideal sensor would approach zero specific force even though its coordinate acceleration is
gravity. Those statements are about the declared ideal measurement equation, not a bench setup.

The fixed multitone `eta_b` is mean-removed and normalized so its three-axis vector RMS equals one
before scaling. It makes repeatable plots and checks. It is not a random draw, white Gaussian
process, standard-deviation claim, PSD, bandwidth model, vibration environment, quantization model,
or identified sensor data.

## Lever 1: gyro bias

Hold accelerometer error at `0.15 m/s^2` vector RMS and sweep
`b_q=[-0.50,-0.25,0,0.25,0.50] deg/s`. Truth, attitude matrices, ideal specific force, additive
accelerometer error, and complete/broken accelerometer measurements remain identical. Only measured
gyro rate and its integrated angle change.

The final angle errors are exactly `[-4,-2,0,2,4] deg` because the horizon is `8 s`. The sign of
the drift follows the bias sign. Mechanism-first explanation: integration accumulates the constant
rate offset at every update; it does not average the offset away.

## Lever 2: accelerometer teaching-error magnitude

Reset gyro bias to `0.20 deg/s` and sweep vector RMS through
`[0,0.05,0.15,0.30,0.50] m/s^2`. Truth, attitude matrices, ideal specific force, gyro rate, and
gyro-integrated angle remain identical. Only the shared fixed error shape is rescaled.

The measured vector RMS of `f_measured-f_ideal` equals the selected value. At zero, measured and
ideal specific force are exactly equal. Mechanism-first explanation: the lever multiplies a fixed
unit-vector-RMS waveform. It changes error amplitude without changing its samples, spectrum, truth,
or any gyro quantity. The UI reset restores the exact baseline rather than relying on slider
placement.

## Limiting cases and invariants

- Zero gyro bias gives exact measured-rate truth and exact integrated-angle truth.
- Zero accelerometer error gives exact ideal specific force.
- Zero bias and zero accelerometer error form the fully ideal measurement limit.
- Before the maneuver and after its return to level, pitch rate and coordinate acceleration are
  zero, while supported-level ideal specific force remains `-g` on body z-down.
- Every `C_b_to_n` is orthonormal with determinant one.
- Transforming ideal body specific force back to NED and adding gravity reconstructs NED coordinate
  acceleration: `C_b_to_n*f_b+g_n=a_n`.
- Each call performs exactly 801 synchronous samples and 800 finite updates. It retains no state.

## Deliberately broken gravity omission

The broken comparison replaces `C_n_to_b*(a_n-g_n)` with `C_n_to_b*a_n`. It preserves time, truth,
attitude, gyro, and additive accelerometer error. At supported level rest it reports zero ideal
specific force rather than `-g` on body z-down. Across the whole maneuver, the magnitude of the
complete-minus-broken measurement is exactly `9.80665 m/s^2`; the shared additive error cancels.

The symptom is finite, smooth, and tempting because the broken trace resembles coordinate
acceleration. The violated assumption is the definition of accelerometer measurement. Passing this
trace to a later estimator would force that estimator to explain away gravity or corrupt attitude
and acceleration estimates. P11 demonstrates the measurement-model defect but does not implement
that estimator.

## Common misconceptions

- A gyro measures angular rate; angle here appears only after an explicit numerical integration.
- A constant gyro bias is not zero-mean noise and does not disappear under integration.
- An accelerometer does not directly report navigation-frame coordinate acceleration.
- A stationary supported accelerometer is not in the same specific-force condition as free fall.
- The sign of the supported-level reading depends on the declared axis convention; this module uses
  body z-down and NED.
- Changing noise magnitude must not change truth, attitude, or gyro output.
- The deterministic multitone is a teaching error, not evidence of stochastic or hardware fidelity.
- Smooth and finite sensor output can still encode the wrong physics.
- P11 does not add quantization, saturation, bias drift, scale factor, misalignment, bandwidth,
  latency, vibration, calibration, GPS, fusion, closed-loop control, or fault management.

## Evidence boundary

Static source inspection and an independent Python equation oracle can establish structure and
simulated reference behavior. MATLAB syntax execution, MATLAB numerical behavior, Live Editor
order, figures, `uifigure` callbacks, UI cleanup, instructional effectiveness, sensor hardware,
bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, and production behavior require
separate named evidence and are not implied here.
