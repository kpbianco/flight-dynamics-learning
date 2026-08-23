# P18 checks: Follow Waypoints

## Guiding question

What inputs, observable effects, and failure modes matter when you follow Waypoints?

Answer from the observed route–bearing–course-response–motion–arrival chain, not from MATLAB
syntax. A complete answer names the ordered stationary North/East waypoints, ideal navigation
position, active target, course convention, arrival radius, response gain and rate limit, route and
course observables, and the risk of using a bearing inconsistent with the declared axes.

## What to observe

Start with `model(30,0.8,1)`:

```text
Delta N     = N_waypoint - N_estimate
Delta E     = E_waypoint - E_estimate
chi_command = atan2(Delta E, Delta N)
e_chi       = wrap(chi_command - chi)
chi_dot     = sat(0.8 e_chi, +/-12 deg/s)
North_dot   = 25 cos(chi)
East_dot    = 25 sin(chi)
capture     = hypot(Delta N,Delta E) <= 30 m
```

The fixed route contains five rows and four legs totaling `1350 m`. Correct mode captures the four
target rows at `[14.8,29.4,41.7,59.2] s`, propagates `1480 m`, and saturates course rate for
`223/592` active samples. Planned-leg cross-track error takes both signs as the finite-rate response
cuts different corners; its RMS is about `52.270 m` and its peak magnitude is about `112.033 m`.
The padded state after final capture is fixed-size recorder bookkeeping, not continued aircraft
motion.

## Controlled levers

1. Reset gain to `0.8 1/s` and correct bearing mode, then sweep arrival radius through
   `[10 20 30 50 80] m`. Route, speed, response, grid, and bearing stay fixed. Every case completes;
   larger circles switch earlier and reduce flown distance while accepting each waypoint farther
   away.
2. Reset radius to `30 m` and correct bearing mode, then sweep response gain through
   `[0 0.2 0.4 0.8 1.2] 1/s`. Route, speed, arrival rule, grid, and bearing stay fixed. Zero gain
   cannot turn after W2; positive gain improves corner response until the fixed rate clamp dominates.

Use the exact interactive reset between sweeps. Moving both levers together hides whether a changed
corner came from the switching boundary or course response.

## Deliberately broken case

Compare:

```matlab
correct=model(30,0.8,1);
broken=model(30,0.8,-1);
```

Both calls retain identical waypoints, initial position, ideal-feedback definition, speed, gain,
radius, arrival-rule and propagation equations, rate limit, and grid. Their resulting position
histories diverge. Correct mode computes
`atan2(Delta East,Delta North)`; broken mode swaps those two components. The first target is due
North, so the commands are `0 deg` and `+90 deg`, respectively. Broken mode captures no target and
misses W2 by about `296.869 m`.

The violated assumption is that the displacement components passed to `atan2` agree with course
measured clockwise from North. Controller tracking error cannot diagnose this fault by itself
because the response may track the wrong command accurately.

## Executable invariants

`run_checks.m` independently requires:

- deterministic repeated baseline equality, 1001 fixed samples, 1000 allocated intervals, and the
  exact five-row route, four leg lengths, `1350 m` planned distance, `25 m/s` speed, `0.1 s` step,
  `12 deg/s` rate limit, and nonoverlapping allowed arrival circles;
- reconstruction of every correct bearing, wrapped course error, unclamped and bounded rate,
  North/East propagation, active-leg along/cross coordinates, Pythagorean target range, ordered
  capture event, active index, terminal hold, and metric;
- exact baseline capture indices/times, capture ranges/positions, completion, active path distance,
  course-error RMS, saturation count/fraction, signed cross-track RMS/peak, final-target range, and
  turn-radius scale;
- inclusive arrival-boundary, zero-error/zero-rate, half-open angle-wrap, constant-step, and zero-gain
  limiting cases;
- two five-point sweeps that preserve every nonlever route, boundary, initial-state, speed, limit,
  and grid field while reconstructing all displayed completion, capture, distance, and saturation
  metrics from retained histories;
- correct-versus-broken equality of every nonbearing input, an immediate `0` versus `+90 deg`
  command, no broken captures or index advance, large first-target miss, exact stateless rollback,
  and baseline recovery;
- rejected below/above-range, nonscalar, complex, `NaN`, `Inf`, and invalid-mode inputs followed by
  an exact valid baseline;
- accepted corners and a capped representative grid with finite, fixed-size histories, monotonic
  in-range active indices, bounded error/rate/position/path length, and no skipped waypoints;
- compatibility with the declared base-MATLAB synchronous stateless scalar interface and explicit
  separation from P17 runtime arrays and P19 moving-target pursuit.

The model creates no background task, external resource, timer, future, worker, callback loop, or
input-sized allocation, so computational timeout and cancellation transitions are not applicable.
Fixed 1001-sample histories, five route rows, bounded public inputs, and capped 8/18-case matrices
are the applicable resource gates. It changes no learner data, score, database, service, schema,
migration, or backup/restore path.

MATLAB runtime execution remains unperformed until these checks run on a named MATLAB environment.

## Interpretation questions

1. Why does course clockwise from North require `atan2(Delta East,Delta North)`?
2. What information must arrive from navigation before waypoint guidance can form a bearing?
3. Why is only one route row active, even if the vehicle passes close to a later waypoint?
4. What does arrival radius change, and what does it leave unchanged?
5. Why can a larger arrival radius shorten the path while reducing spatial closeness?
6. What are the units of `K_chi`, `e_chi`, and `K_chi e_chi`?
7. Why does increasing gain eventually stop reducing completion time substantially?
8. What does the zero-gain limit prove about waypoint management versus turn response?
9. Which retained fields prove the broken case changes only the N/E bearing convention?
10. Why could a small tracking-error metric coexist with a completely wrong route?
11. Why is the terminal fixed-size hold not a claim that the aircraft stopped?
12. Which omitted mechanisms prevent pursuit, aircraft, robustness, safety, or certification claims?

## Teach-back

In two sentences, answer the guiding question. First trace an ideal P17-style North/East position
and ordered active waypoint through displacement, clockwise-from-North bearing, shortest course
error, bounded course response, planar motion, and inclusive arrival. Second explain how arrival
radius and course-response gain change different observables, then diagnose why swapping the
North/East bearing components sends the vehicle toward the wrong cardinal direction.
