# P18 lesson: Follow Waypoints

## Guiding question

What inputs, observable effects, and failure modes matter when you follow Waypoints?

## Compound P17 into a guidance question

P17 separated a high-rate INS prediction from lower-rate GPS corrections and produced a transparent
North position/velocity estimate. P18 begins downstream of that estimator: what should guidance do
with position and an ordered route?

```text
P17 navigation estimate -> active stationary waypoint -> N/E displacement
                        -> course bearing -> bounded response -> new position
                        -> 2-D arrival decision -> next waypoint
```

The connection is conceptual rather than current API compatibility. P18 does not consume P17
histories. It uses the exact simulated two-dimensional North/East position as an ideal estimate so
navigation error cannot hide the waypoint-management mechanism. P14 previously connected heading
error to bank and turn rate; P18 reduces that inner behavior to a bounded course-rate response and
does not consume P14 arrays or run bank dynamics.

## Give route management and response different jobs

The route is an ordered five-row `[North East]` table in meters:

```text
W1 = [  0,   0]   W2 = [400,   0]   W3 = [400, 300]
W4 = [100, 300]   W5 = [100, 650]
```

Only one target is active. Guidance subtracts the estimated position from that waypoint and forms a
course measured clockwise from North:

```text
Delta N     = N_waypoint - N_estimate
Delta E     = E_waypoint - E_estimate
chi_command = atan2(Delta E, Delta N)
```

This argument order follows directly from the convention: due North is `atan2(0,positive)=0 deg`,
and due East is `atan2(positive,0)=+90 deg`. It is not a syntax preference.

The course state is continuous. Feedback uses the shortest circular error:

```text
e_chi   = wrap(chi_command - chi) in [-180,180) deg
chi_dot = sat(K_chi e_chi, +/-12 deg/s)
```

`K_chi` has units `1/s`, so multiplying radians of course error gives `rad/s`. The rate clamp then
makes limited turn authority explicit. With zero wind and fixed `V_g=25 m/s`, planar kinematics are

```text
North_dot = V_g cos(chi)
East_dot  = V_g sin(chi)
```

At each retained sample, the manager tests the Euclidean range to the active waypoint. It advances
one row only when `hypot(Delta N,Delta E)<=R`. Waypoint order matters: proximity to a later row
cannot skip the active target.

## Baseline: follow causal order

Use `model(30,0.8,1)` on the fixed `0:0.1:100 s` grid. At each active sample:

1. test the current N/E position against the current target's inclusive arrival circle;
2. if captured, advance exactly one target before forming the new command;
3. subtract position from the active stationary waypoint;
4. compute clockwise-from-North bearing with `atan2(Delta E,Delta N)`;
5. wrap only the course error and clamp `K_chi e_chi` to `+/-12 deg/s`;
6. propagate North, East, and continuous course one fixed step.

The target capture times are `14.8`, `29.4`, `41.7`, and `59.2 s`. Because the manager accepts a
circle rather than requiring exact coincidence, their sampled ranges are about `30`, `28.613`,
`29.414`, and `27.828 m`. The flown distance is `1480 m`, longer than the `1350 m` centerline route
because finite course-rate response widens corners. The course-error RMS is about `33.914 deg`, and
the course-rate command is saturated for `223/592` active samples.

Those figures are retained independent simulated-oracle references. First inspect the planned and
flown route. Then inspect commanded versus actual course. Only then use range, index, course error,
and rate to explain why each corner looks the way it does.

## Lever 1: arrival radius

Hold `K_chi=0.8 1/s` and correct bearing mode, then sweep
`R=[10,20,30,50,80] m`.

- Waypoints, initial state, speed, rate limit, gain, grid, and bearing equation stay fixed.
- Every case captures all four targets.
- Completion time decreases from `63.1 s` to `50.0 s`.
- Flown distance decreases from `1577.5 m` to `1250 m`.
- Each capture is inside the selected radius but no more than one `2.5 m` step inside it.

Mechanism first: the radius is a decision boundary, not turn authority. A larger circle declares
the current target complete sooner and points toward the next row sooner, so the recorded route cuts
corners. That can be useful, but it trades spatial closeness for earlier switching; it is not a free
accuracy improvement.

## Lever 2: course-response gain

Reset `R=30 m`, keep correct bearing mode, and sweep
`K_chi=[0,0.2,0.4,0.8,1.2] 1/s`.

- Waypoints, initial state, speed, rate limit, arrival rule, radius, and grid stay fixed.
- At zero gain, course rate is exactly zero. The vehicle reaches W2 while flying North, then cannot
  turn East toward W3 and does not complete the route.
- Positive-gain cases complete at approximately `[65.6,60.0,59.2,59.1] s`.
- Higher gain asks for faster initial turns, but the fixed `12 deg/s` clamp makes the benefit
  plateau while the saturation fraction rises.

Mechanism first: gain changes how course error becomes demanded rate. It does not alter the route or
the range threshold. Once demanded rate reaches the clamp, still more gain cannot add course-rate
authority.

## Limiting geometry and resource checks

- A zero course error produces zero unconstrained and bounded rate.
- The half-open wrap maps a `+180 deg` difference to `-180 deg`; it never commands a `>180 deg`
  turn error.
- The inclusive boundary accepts exactly `range=R` and rejects the next representable value above.
- `K_chi=0` is an exact open-course-loop limit: `North=25t`, East and course remain zero, only W2
  is captured, and final North is `2500 m`.
- Every moving interval covers exactly `V_g dt=2.5 m`.
- The saturated kinematic turn-radius scale is `V_g/chi_dot_max`, about `119.366 m`.
- The model always returns 1001-sample histories and considers at most five fixed route rows. After
  final acceptance it pads the terminal state only for fixed-size comparison; active-only metrics
  exclude that nonphysical recorder hold.

These limits distinguish a route manager that correctly reached a boundary from a simulation that
silently stopped updating.

## Deliberately broken: swap North and East

Broken mode changes only the bearing arguments:

```text
correct = atan2(Delta East,  Delta North)
broken  = atan2(Delta North, Delta East)
```

For the first due-North target, correct mode commands `0 deg`; broken mode commands `+90 deg`.
The broken path turns East, captures no target waypoint, stays at active index 2, and misses W2 by
about `296.869 m` at closest approach. Its final position after `100 s` is approximately
`[-1429.64,1817.97] m` in `[North,East]`.

Do not diagnose this as inadequate course gain: correct and broken runs use the same gain and rate
limit. Do not diagnose it as navigation noise, waypoint corruption, or arrival-radius failure:
their initial position, ideal-feedback definition, waypoint table, and range-test equation are
identical. Their position and evaluated-range histories diverge after the changed bearing command.
Do not accept a small course tracking error as success; the response can track a geometrically wrong
command. The recognizable symptom is motion toward the wrong cardinal direction before the first
waypoint can be reached.

## Common misconceptions

- `atan2(East,North)` is required here because course is clockwise from North; Cartesian
  `atan2(y,x)` mnemonics are unsafe until axes and angle convention are declared.
- Wrapping course error does not discard vehicle rotation; the internal course state remains
  continuous.
- Arrival radius is not path-following gain, turn radius, navigation accuracy, or obstacle clearance.
- More gain cannot overcome a hard course-rate limit.
- Reaching an arrival circle is not the same as flying through the exact waypoint coordinate.
- A shorter flown path can reflect more corner cutting rather than better tracking.
- An ideal navigation input does not prove P17 estimator accuracy or make navigation faults harmless.
- Fixed stationary-waypoint direct-to bearing is not moving-target pursuit, lead, interception, or
  proportional navigation; those concepts remain for P19.
- A planar zero-wind constant-speed trace is not aircraft, autopilot, envelope, robustness, or
  certification evidence.

## Evidence boundary

Static source inspection and an independent standard-library Python oracle can establish artifact
structure, deterministic equation behavior, two-sweep isolation, bounds, and broken-case symptoms.
MATLAB syntax execution, MATLAB numerical behavior, Live Editor order, figures, `uifigure`
callbacks/reset, learner understanding, guidance fidelity, navigation fidelity, aircraft behavior,
bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, staging, and production behavior
require separate named evidence and are not implied.
