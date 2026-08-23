# P17 checks: Fuse INS and GPS

## Guiding question

What inputs, observable effects, and failure modes matter when you fuse INS and GPS?

Answer from the observed sensor–prediction–innovation–gate–correction chain, not from MATLAB syntax.
A complete answer names the gravity-compensated INS acceleration input, constant residual bias,
GPS position fixes and nominal error, update rates, predicted position/velocity, innovation,
alpha-beta gains, gate decision, fused error, and the risk of accepting an implausible fix.

## What to observe

Start with `model(0.04,1,1)`:

```text
p_minus = p_previous + v_previous dt + 0.5 a_INS dt^2
v_minus = v_previous + a_INS dt
r_GPS   = z_GPS - p_minus
accept  = abs(r_GPS) <= 25 m
p_plus  = p_minus + 0.45 r_GPS
v_plus  = v_minus + (0.12/1 s) r_GPS
```

The INS updates every `0.02 s`; GPS fixes occur only at integer seconds `1` through `60`. The
baseline INS-only final errors are `72 m` and `2.4 m/s`. Correct fusion accepts 59 fixes, rejects
the fixed outlier once, and applies exactly zero correction at that rejected sample.

## Controlled levers

1. Reset GPS position-error RMS to `1 m` and gated mode, then sweep INS bias through
   `[0 0.02 0.04 0.06 0.08] m/s^2`. Truth and GPS data remain fixed. INS-only final position
   follows `0.5*b_a*60^2`; velocity follows `b_a*60`. Fused innovations carry the accumulating
   prediction error to the next absolute fix.
2. Reset INS bias to `0.04 m/s^2`, then sweep nominal GPS error RMS through `[0 0.5 1 2 4] m`.
   Truth, INS acceleration, and INS-only histories remain fixed. The nominal GPS error has exactly
   the selected RMS, and accepted corrections transfer more of it into the fused estimate.

Use the exact interactive reset between sweeps. Moving both levers together hides which sensor is
responsible for a changed prediction or correction.

## Deliberately broken case

Compare:

```matlab
correct=model(0.04,1,1);
broken=model(0.04,1,-1);
```

Both calls retain identical truth, INS acceleration, GPS nominal error, outlier, grid, gains, and
initial state. Correct mode rejects the roughly `79.34 m` innovation at `30 s`. Broken mode
disables only that gate, accepts all 60 fixes, and applies a large position and velocity correction.
The position error then grows until the next fix and reaches roughly `44 m` peak.

The violated assumption is that every delivered GPS fix is credible enough to update navigation
state. Later nominal fixes pull the state back but do not constitute fault detection, isolation, or
safe recovery.

## Executable invariants

`run_checks.m` independently requires:

- deterministic repeated baseline equality and fixed 3001-sample/3000-interval resources;
- exact truth schedule, final truth, sensor histories, 60 GPS fix times, no `t=0` fix, and nominal
  GPS mean/RMS normalization before the outlier;
- reconstruction of every truth and INS-only update plus the exact `b_a*t` and
  `0.5*b_a*t^2` dead-reckoning limits;
- reconstruction of every predicted state, innovation, inclusive gate decision, position/velocity
  correction, and corrected state;
- correct baseline counts, signed numerical references, and zero correction at the rejected fix;
- fully ideal, zero-bias, zero-GPS-error, INS-only, bias-sign, and exact gate-boundary limits;
- noise-free `+/-0.08 m/s^2` bias cases with opposite INS/fused error and correction histories,
  identical exogenous GPS data and gate decisions, and a `0.55` position-error contraction at every
  accepted fix;
- two five-point sweeps that preserve every nonlever truth/sensor history and reconstruct all
  reported RMS/peak metrics from retained histories;
- isolated correct-versus-broken equality before the outlier, identical exogenous histories for the
  whole run, accept-all state jump, later-error symptom, stateless rollback, and baseline recovery;
- rejected below/above-range, nonscalar, complex, `NaN`, `Inf`, and invalid-mode inputs;
- accepted corners and a capped representative grid with finite, fixed-size histories and bounded
  gains, corrections, sensor values, and errors;
- compatibility with the declared base-MATLAB synchronous stateless interface.

The model creates no background task, external resource, timer, future, worker, callback loop, or
input-sized allocation, so computational timeout and cancellation paths are not applicable. Fixed
3001-sample histories, 60 fixes, bounded public inputs, and capped 12/27-case matrices are the
applicable resource gates. It changes no learner data, score, database, service, schema, migration,
or backup/restore path.

MATLAB runtime execution remains unperformed until these checks run on a named MATLAB environment.

## Interpretation questions

1. Why does a constant acceleration bias create linear velocity error but quadratic position error?
2. What does the INS contribute between one-Hz GPS fixes?
3. Why is GPS position compared with the predicted state rather than the previous corrected state?
4. What are the units of `alpha` and `beta/T_GPS`?
5. Why does accepted GPS error appear in both corrected position and velocity?
6. Why can ideal INS make GPS corrections unnecessary in this declared limiting case?
7. Why is an ignored fix different from a gate-rejected fix?
8. Which histories prove the broken case changes only the innovation-gate decision?
9. Why does later reconvergence not make accepting the outlier safe?
10. Which omitted mechanisms prevent navigation-integrity, receiver, aircraft, or certification claims?

## Teach-back

In two sentences, answer the guiding question. First trace gravity-compensated INS acceleration and
bias through high-rate position/velocity prediction, then trace a lower-rate GPS position fix
through innovation, the inclusive gate, and alpha-beta correction. Second explain how the two
sensor-error sweeps expose drift-versus-measurement tradeoffs and diagnose why accepting an
implausible GPS outlier creates a navigation-state jump.
