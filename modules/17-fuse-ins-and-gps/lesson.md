# P17 lesson: Fuse INS and GPS

## Guiding question

What inputs, observable effects, and failure modes matter when you fuse INS and GPS?

## Compound P16 into a navigation question

P16 closed the autopilot phase by scheduling a feedback controller across flight conditions. That
controller, like the P13–P15 teaching controllers, treated its state as known. P17 starts the
navigation phase with the missing question: how can a vehicle maintain a useful position and
velocity estimate when its sensors have different rates, strengths, and failure modes?

```text
P11 sensor meanings -> attitude/gravity compensation -> P17 navigation estimate -> P18 guidance
P16 state feedback ------------------------------------------------------^ conceptual need
```

The connection is conceptual rather than current API compatibility. P17 does not consume P16 roll
histories, P12 trajectories, or P11 sensor arrays. It declares a one-dimensional North truth and
assumes an upstream mechanization has already turned inertial measurements into a
gravity-compensated North acceleration. This keeps the prediction/correction mechanism visible.

## Give each sensor one job

The simplified INS supplies high-rate continuity. Every `0.02 s`, it propagates position and
velocity with its measured acceleration:

```text
p_minus(k) = p_plus(k-1) + v_plus(k-1) dt + 0.5 a_INS(k-1) dt^2
v_minus(k) = v_plus(k-1) + a_INS(k-1) dt
a_INS      = a_truth + b_a
```

The superscript idea is more important than the notation: minus means the state predicted before
the current GPS correction; plus means the state after that correction. A constant residual
acceleration bias integrates once into velocity error and twice into position error. With exact
initial state and no GPS corrections,

```text
e_v,INS(t) = b_a t
e_p,INS(t) = 0.5 b_a t^2
```

GPS supplies a lower-rate absolute position. It does not directly replace the complete high-rate
state. At one-second fix times the estimator forms a residual, or innovation:

```text
r_GPS = z_GPS - p_minus
```

The sign matters. A positive residual says the GPS fix lies farther North than the prediction, so
the correction should move estimated North position and velocity in the positive direction.

## Make fusion an inspectable correction

For an accepted fix, the model uses fixed alpha-beta gains:

```text
p_plus = p_minus + alpha r_GPS
v_plus = v_minus + (beta/T_GPS) r_GPS
alpha  = 0.45
beta   = 0.12
T_GPS  = 1 s
```

`alpha` is dimensionless. `beta/T_GPS` has units `1/s`, so multiplying the position residual gives
a velocity correction in `m/s`. The estimator is not using `beta/dt`; the one-second GPS period,
not the `0.02 s` INS interval, scales this discrete velocity correction.

This fixed-gain filter exposes the central trade. INS prediction bridges the gap between absolute
fixes but accumulates bias. GPS correction bounds that drift but injects a portion of GPS error.
The model deliberately does not call a Kalman filter or hide covariance tuning behind a toolbox.

## Baseline: follow causal order

Truth starts at `0 m`, moving North at `20 m/s`. It accelerates at `+0.5 m/s^2` from `5` to
`15 s`, coasts, decelerates at `-0.5 m/s^2` from `25` to `35 s`, then coasts to `60 s`. The final
truth is `1300 m` at `20 m/s`.

The baseline residual INS bias is `+0.04 m/s^2`. The model's 60 nominal GPS errors come from a
fixed multitone, mean-removed and normalized before scaling to exactly `1 m` RMS. No random draw is
made. At each sample, inspect this order:

1. propagate truth and INS acceleration through constant-acceleration kinematics;
2. predict estimator position and velocity from the previous corrected state;
3. if a GPS fix exists, compute its position innovation;
4. compare the innovation magnitude with the inclusive `25 m` gate;
5. apply position and velocity corrections only when accepted.

Position and velocity cannot know a future GPS fix. At a fix, the predicted state exists first;
the innovation compares that prediction with the measurement; the correction then creates the
retained fused state.

Without GPS, baseline bias creates exactly `72 m` final position error and `2.4 m/s` final velocity
error. Gated fusion instead has about `1.180878 m` position RMS and `2.385487 m` peak absolute
position error. That improvement is the visible effect of absolute correction, not proof of an
optimal filter or receiver accuracy.

## Lever 1: residual INS acceleration bias

Hold GPS error at `1 m` RMS and sweep
`b_a=[0,0.02,0.04,0.06,0.08] m/s^2`.

- Truth, GPS fix times, nominal GPS error, outlier, gains, gate, grid, and initial state remain fixed.
- INS-only final position errors become exactly `[0,36,72,108,144] m`.
- INS-only final velocity errors become exactly `[0,1.2,2.4,3.6,4.8] m/s`.
- Fused error remains bounded by accepted fixes, but larger bias loads each prediction and changes
  the residual that the next GPS fix must correct.

Mechanism first: bias is not a one-time position offset. It enters acceleration every INS update,
so integration determines the time powers in the drift. GPS corrections reset neither the physical
bias nor the sensor; they correct its accumulated effect on the estimated state.

## Lever 2: deterministic GPS position error

Reset bias to `0.04 m/s^2` and sweep nominal GPS error RMS through `[0,0.5,1,2,4] m`.

- Truth, INS acceleration, INS-only position and velocity, fix times, gains, and gate remain fixed.
- The shared unit-RMS waveform is simply rescaled; its measured nominal RMS equals the lever.
- Correct-mode fused position RMS rises approximately
  `[0.254244,0.625401,1.180878,2.330609,4.650654] m`.

Mechanism first: an accepted absolute fix is informative but not perfect. Multiplying its residual
by nonzero gains transfers some measurement error into position and velocity. Lower INS drift and
lower GPS error are distinct improvements; changing one must not silently change the other.

## Limiting cases

- `model(0,0,1)` follows truth exactly. Every nominal innovation is zero, and the fixed outlier is
  still rejected.
- `model(0,0,0)` also follows truth exactly because a bias-free INS needs no correction in this
  declared one-dimensional truth model; all GPS fixes are classified as ignored, not rejected.
- Mode `0` with nonzero bias reproduces the retained INS-only arrays exactly.
- A bias sign reversal reverses the exact INS-only error histories, while truth and GPS data remain
  unchanged.
- The gate is inclusive: an innovation exactly at `25 m` is accepted by definition.

These limits distinguish a correct zero-error result from a filter that accidentally stopped
processing or changed its truth input.

## Deliberately broken: accept every GPS fix

At `30 s`, every mode receives the same GPS stream with the same added `+80 m` contamination.
Correct fusion sees about `79.34 m` innovation, marks that fix gate-rejected, and applies exactly
zero correction. Broken mode changes only one decision:

```text
correct: accept = abs(r_GPS) <= 25 m
broken:  accept = true
```

The broken state jumps about `35 m` in position and receives about `9.5 m/s` of velocity correction.
It continues moving away until the next fix, then later nominal fixes gradually pull it back. A
finite trace that eventually reconverges does not make the failure safe. The symptom is a large,
causal state jump from an implausible residual; the violated assumption is that every GPS
measurement is valid enough to update navigation state.

Do not diagnose this as INS bias growth: the correct and broken INS histories are identical. Do not
diagnose it as a GPS unit conversion: the measurement stream is identical. Do not call later
corrections fault recovery: the accept-all filter had no detection or isolation action. A real
navigation system needs richer integrity logic, uncertainty modeling, and fault handling.

## Numerical and resource invariants

- Every call retains 3001 samples, 3000 predictions, and 60 scheduled GPS fixes at `1:60 s`.
- No GPS fix exists at `t=0`; the declared initial state is exact.
- Truth, both dead-reckoning histories, every predicted state, innovation, gate decision, accepted
  correction, and corrected state can be independently reconstructed.
- Nominal GPS error has zero mean and exactly the selected RMS over fix samples before contamination.
- Correct baseline accepts 59 fixes, gate-rejects one, and ignores none; INS-only ignores all 60.
- Correct and broken histories are identical before the outlier update; exogenous sensor histories
  remain identical for the complete run.
- All public inputs are bounded, every history has fixed size, and a capped representative grid is
  finite and deterministic.
- Invalid, nonscalar, complex, `NaN`, `Inf`, and invalid-mode calls reject before calculation; the
  stateless model reproduces baseline afterward.

## Common misconceptions

- An INS does not provide drift-free position merely because it updates quickly.
- A GPS fix does not need to overwrite position completely to correct drift.
- Position innovation can update velocity because position residual accumulated over a known fix interval.
- `beta/T_GPS` and `beta/dt_INS` are not interchangeable.
- A correction that reduces error now can still inject measurement error.
- A rejected outlier is not a GPS dropout; the fix arrived and failed a declared credibility test.
- An innovation gate is not spoofing protection, receiver integrity, or a certification argument.
- Deterministic multitone error is not white Gaussian noise or identified receiver performance.
- One-dimensional North motion is not a complete inertial navigation mechanization.
- P17 does not implement covariance propagation, bias estimation, attitude, geodesy, satellite
  measurements, bus timing, fault management, guidance, or closed-loop aircraft behavior.

## Evidence boundary

Static source inspection and an independent standard-library Python equation oracle can establish
artifact structure and simulated equation behavior. MATLAB syntax execution, MATLAB numerical
behavior, Live Editor order, figures, `uifigure` callbacks, learner understanding, estimator or
receiver fidelity, aircraft behavior, bench, HIL, field, RT1/RT2, Unreal, signing, release,
deployment, staging, and production behavior require separate named evidence and are not implied.
