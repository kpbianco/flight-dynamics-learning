# P17 — Fuse INS and GPS

**Track:** Flight Dynamics and Aerospace GNC

**Phase 5:** Navigation and guidance

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you fuse INS and GPS?

## Physical mental model

P16 ended the autopilot phase with feedback laws that assumed usable state information. P17 begins
navigation by exposing where a North position and velocity estimate can come from. A simplified INS
propagates at `50 Hz`; GPS supplies an absolute position fix at `1 Hz`:

```text
p_minus(k) = p_plus(k-1) + v_plus(k-1) dt + 0.5 a_INS(k-1) dt^2
v_minus(k) = v_plus(k-1) + a_INS(k-1) dt
r_GPS      = z_GPS - p_minus
p_plus     = p_minus + alpha r_GPS
v_plus     = v_minus + (beta/T_GPS) r_GPS
```

The fixed gains are `alpha=0.45` and `beta=0.12`. A fix is used only when
`|r_GPS| <= 25 m`. This is a transparent alpha-beta teaching filter: prediction preserves high-rate
motion between fixes, while correction anchors integrated drift to an absolute measurement.

North position, velocity, acceleration, innovation, and corrections are positive North. The INS
input is already a gravity-compensated North acceleration. P11 introduced gyro and accelerometer
measurement meanings, and P12 reinforced NED/frame conventions; P17 deliberately omits the attitude
and gravity-compensation mechanization between those measurements and this horizontal acceleration.

## Deterministic baseline

The fixed grid is `0:0.02:60 s`: 3001 samples and 3000 constant-acceleration predictions. Truth
starts at `0 m` and `20 m/s`, accelerates at `+0.5 m/s^2` from `5` to `15 s`, then at
`-0.5 m/s^2` from `25` to `35 s`. It ends at `1300 m` and `20 m/s`.

The baseline INS has a constant `+0.04 m/s^2` residual acceleration bias. If GPS is ignored, the
velocity and position errors close exactly to

```text
e_v,INS(t) = b_a t
e_p,INS(t) = 0.5 b_a t^2
```

so the final dead-reckoned errors are `2.4 m/s` and `72 m`. Nominal GPS error is a fixed,
mean-removed multitone normalized to `1 m` RMS over the 60 fix times. It is replayable teaching
data, not a random draw, white-noise model, receiver specification, or hardware record.

Gated fusion accepts 59 nominal fixes and rejects the single contaminated fix at `30 s`. An
independent standard-library equation oracle gives about `1.180878 m` fused position RMS,
`2.385487 m` peak absolute position error, `0.136264 m` final position error, and
`-0.115398 m/s` final velocity error. These are deterministic simulated references, not MATLAB
runtime, UI, receiver, aircraft, bench, HIL, or field evidence.

## Two independent levers

1. Hold nominal GPS error at `1 m` RMS and sweep INS acceleration bias through
   `[0,0.02,0.04,0.06,0.08] m/s^2`. INS-only final position error follows
   `[0,36,72,108,144] m`; final velocity error follows `[0,1.2,2.4,3.6,4.8] m/s`.
   The gated estimate stays bounded by fixes, but its prediction and innovation workload grows.
2. Reset bias to `0.04 m/s^2` and sweep deterministic GPS position-error RMS through
   `[0,0.5,1,2,4] m`. Truth, INS acceleration, and INS-only histories remain fixed. The nominal
   measurement error has exactly the selected RMS, and the fused position RMS rises from about
   `0.254244 m` to `4.650654 m` because accepted corrections carry GPS error into the estimate.

The fully ideal `model(0,0,1)` limit reproduces truth exactly. It still rejects the fixed `+80 m`
contamination, which shows that gate behavior is independent of nominal sensor-error magnitude.
Mode `0` is the exact INS-only limit: all 60 GPS fixes are available but intentionally ignored.

## Deliberately broken accept-all fusion

Every mode receives the same truth, INS acceleration, nominal GPS error, and an added `+80 m` GPS
outlier at `30 s`. Correct mode sees an innovation of about `79.3378 m`, rejects it, and applies
zero position and velocity correction. Broken mode disables only the innovation gate and accepts
all 60 fixes.

The broken estimator immediately applies `alpha*r_GPS` to position and
`(beta/T_GPS)*r_GPS` to velocity. The position error jumps to about `34.73 m`, grows further before
the next fix, reaches about `43.95 m` peak, and has about `6.52 m` RMS over the trace. Later nominal
fixes pull the estimate back, but that is not evidence that the outlier was harmless. The violated
assumption is that every GPS fix is credible enough to update the state.

## Scope and prerequisite boundary

P16 supplies the conceptual need for state estimates used by control, not a runtime input. P11 and
P12 supply sensor and NED meanings, not compatible arrays. P17 accepts no prior module history and
does not run an aircraft, controller, attitude solution, or GPS receiver. Its output is a
one-dimensional teaching estimate that prepares the position/velocity ideas used by P18 guidance.

The model omits three-dimensional attitude and navigation, Earth rotation, transport rate, gravity
modeling, coning/sculling, lever arms, clock states, pseudoranges, satellite geometry, geodesy,
correlated noise, covariance propagation, bias estimation, Kalman tuning, integrity monitoring,
dropouts, latency, multipath, spoofing, jamming, terrain, wind, closed-loop guidance, uncertainty,
and certification. The fixed innovation gate is a visible fault-isolation device, not a navigation
integrity guarantee.

## Run

From MATLAB at the repository root:

```matlab
launch_lesson("P17")
run_module_checks("P17")
```

The implementation uses base MATLAB arithmetic, fixed arrays, explicit loops, labeled plots, and
`uifigure` controls. It calls no filter, navigation, sensor, optimization, or control toolbox; uses
no random source, file, network, device, timer, future, or parallel worker; and retains no state.
There is no background task to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and mechanism-first narrative.
- `model.m` — guarded truth, INS, GPS, prediction, gate, correction, and metric calculations.
- `experiment.m` — baseline, two independent sweeps, limits, and broken accept-all fusion.
- `interactive.m` — bias/error controls, mode selector, exact reset, and immediate views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, observations, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and an independent Python equation oracle can establish structure and
simulated reference behavior without MATLAB. They do not establish MATLAB parsing or execution,
figures, callbacks, learner understanding, navigation fidelity, hardware, HIL, field, release,
deployment, or production behavior.
