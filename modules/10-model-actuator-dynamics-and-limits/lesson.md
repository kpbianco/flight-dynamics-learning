# P10 lesson: Model Actuator Dynamics and Limits

## Guiding question

What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?

## Compounds on P09

P09 propagated a complete rigid-body state from prescribed body force and moment. Its equations saw
the moment that reached the vehicle, not the upstream flight-computer command. P10 opens that
boundary. A generic signed control-surface actuator turns a deflection command into a delivered
deflection, and a declared gain turns that deflection into a conceptual body-y moment ledger.

```text
deflection command (deg)
          |
          v
 position-envelope target saturation (+/-15 deg)
          |
          v
 first-order lag (tau)
          |
          v
 motion rate stop (+/-rate_limit deg/s)
          |
          v
 delivered deflection delta (deg) ---> conceptual body-y M_y ledger (N*m)
```

This is a transparent teaching actuator, not an elevator, hydraulic, electromechanical, or
identified-aircraft model. Positive deflection produces positive pitch moment only through the
declared `80 N*m/deg` teaching gain.

This is a conceptual connection, not a directly compatible package interface. P09's public
`model(forcePulseScale,momentPulseScale)` function constructs its own three-axis pulse on a different
grid; it does not accept P10's arbitrary scalar history, and P10 supplies no adapter. A combined
simulation would need an explicitly designed time-grid and body-moment-vector interface in a later
approved batch.

## Three mechanisms, one state

The actuator has one dynamic state, delivered deflection `delta`. The requested command is clipped
to the symmetric hard-stop envelope before it becomes a feasible target:

```text
delta_command_limited = clip(delta_command, -15 deg, +15 deg)
delta_dot_raw         = (delta_command_limited - delta) / tau
delta_dot             = clip(delta_dot_raw, -rate_limit, +rate_limit)
delta_(k+1)           = clip(delta_k + dt delta_dot_k, -15 deg, +15 deg)
M_y_delivered         = (80 N*m/deg) delta
```

The first clip answers “where may the surface go?” The second answers “how fast may it move?” The
time constant answers “how aggressively does the unsaturated actuator close its remaining error?”
Those are different questions. A larger command does not create more travel after the hard stop,
and a smaller time constant does not create more rate while the rate stop is active.

For the accepted `dt/tau <= 0.2`, each complete Euler update is a partial move toward the already
position-limited target, so it stays inside `+/-15 deg` without contacting the post-update clip. That
last clip remains a defensive guard against future grid, state, or recurrence changes. It is retained
and checked, but it is not presented as an independently active baseline mechanism.

The explicit update uses `dt=0.01 s` over `0:0.01:5 s`: exactly 501 retained samples and 500 bounded
updates for each complete or broken trajectory. The calculation is deterministic, synchronous, and
stateless. It launches no background operation, so timeout and cancellation are not model API
semantics.

## Deterministic command and baseline

The fixed schedule begins at `0 deg`, steps to an infeasible `+25 deg` at `0.5 s`, reverses to
`-25 deg` at `2.0 s`, and finishes at the feasible `+5 deg` command at `3.5 s`. The baseline uses
`tau=0.18 s`, a `45 deg/s` rate limit, and the fixed `+/-15 deg` position envelope.

The hard stop makes both `+25 deg` and `-25 deg` requests feasible only as `+15 deg` and `-15 deg`.
The initial step asks for `83.333 deg/s`, so the actuator first moves at `45 deg/s`. The full
reversal asks for about `-166.646 deg/s`, so the rate stop controls the early reversal even more
strongly. Only after the position error shrinks does the first-order lag regain control.

The position-limit ledger marks samples whose *requested command* lies outside the envelope; its
duration is not surface dwell time at a mechanical stop. It uses the same definition for complete
and broken trajectories, so removing enforcement cannot erase evidence of an infeasible request.

Independent standard-library equation evaluation gives these baseline signatures:

- `501` samples and `500` updates over `5 s`;
- response time `0.45 s` to 90% of the feasible `+15 deg` target after the `+25 deg` request;
- zero crossing `0.34 s` after the full reversal;
- peak delivered rate `45 deg/s`;
- position-request limiting for `3.00 s` and rate limiting for `0.92 s`;
- feasible-command RMS error about `7.70015 deg`;
- peak delivered pitch moment about `1199.706 N*m`.

These are deterministic simulated teaching references, not MATLAB-runtime or identified-actuator
evidence.

## Lever 1: time constant

Reset the rate limit to `45 deg/s`, keep the hard stop at `+/-15 deg`, and sweep
`tau=[0.08,0.12,0.18,0.28,0.40] s`. Command history and both limit values stay fixed. For the same
remaining error, a larger time constant lowers raw unsaturated rate demand. Across the evolving
trajectories, error histories differ, so raw rates need not stay pointwise ordered. The retained
observables are that a larger time constant lengthens the feasible-target response and increases
feasible-command RMS error. It can reduce the duration of rate saturation without tracking better.

Mechanism-first explanation: `tau` divides the remaining error. For a fixed error it directly scales
raw demand; along separate trajectories it also changes the error being divided. While the rate clip
is active, changing `tau` may change hidden demand without changing delivered rate.

## Lever 2: rate authority

Reset `tau` to `0.18 s`, keep the hard stop fixed, and sweep the rate limit through
`[20,30,45,60,80] deg/s`. The requested and position-limited command histories remain identical.
More rate authority shortens the positive response, crosses zero sooner after reversal, and lowers
feasible-command RMS error. The delivered peak rate rises only because this schedule asks for enough
motion to reach each swept stop.

The interactive reset button restores the exact baseline values between these two levers; it avoids
depending on approximate slider placement.

Mechanism-first explanation: during a large reversal the rate stop bounds the slope of deflection
versus time. A rate-limited trace is nearly linear even though the underlying unconstrained actuator
is first order. When the remaining error becomes small enough, the trace bends into the lag response.

## Limiting cases

- With `tau=0.50 s` and `rate_limit=120 deg/s`, the declared command schedule never reaches the rate
  stop. The visible recurrence reduces to the position-limited first-order Euler update.
- Across every accepted corner, the complete candidate update stays inside the target envelope; the
  post-update state clip is therefore a defensive guard on this declared domain.
- Before `0.5 s`, command, feasible command, state, rate, and moment are exactly zero.
- A command already equal to the current deflection gives zero raw rate and zero update.
- An infeasible steady command cannot push the complete actuator beyond `+/-15 deg`, regardless of
  how long it is held.
- Increasing rate authority beyond the raw lag demand has no effect; the rate stop is then inactive.

## Deliberately broken position envelope

The broken comparison keeps the same sample grid, command, time constant, rate limit, recurrence,
and moment gain but omits target saturation and the defensive post-update guard—the two enforcement
points of one position envelope. Its surface moves smoothly and still respects `45 deg/s`, yet
reaches about `24.987 deg`: roughly `9.987 deg` beyond the declared envelope. The mapped peak moment
rises to about `1998.952 N*m`, inventing about `798.952 N*m` beyond the maximum feasible magnitude.
The complete and broken states first diverge at `0.67 s`, while both are still far inside `15 deg`,
because target saturation was masked by the shared rate stop until the complete actuator returned to
lag-governed motion. This is an omitted-envelope failure, not simulated mechanical contact.

The symptom is not numerical instability. It is a finite, plausible-looking output that violates a
physical position constraint. Comparing only rate or smoothness would miss it; compare delivered
deflection against the envelope and trace the unavailable moment into the conceptual rigid-body
boundary.

## Common misconceptions

- Command, position-limited command, and delivered deflection are three different signals.
- A first-order time constant is not a pure time delay; motion begins immediately unless another
  mechanism prevents it.
- Rate limiting bounds slope in `deg/s`; position limiting bounds magnitude in `deg`.
- On this accepted grid/domain, target clipping and a monotone update already retain the state
  envelope; the post-update clip is a defensive guard for changed assumptions, not an active result.
- Smaller `tau` cannot defeat the explicit rate stop. For the same error it raises hidden raw demand,
  but delivered motion remains capped.
- A surface can obey its position and rate limits while still being a poor model of current draw,
  backlash, hinge moment, load-dependent authority, or failure modes not represented here.
- The linear moment gain is a visible interface demonstration, not aerodynamic fidelity.
- A smooth, finite broken trace is not evidence that actuator limits were modeled.
- P10 exposes the conceptual delivered-moment boundary that precedes rigid-body propagation; it does
  not wire into P09's public API or add P11 sensors or P13 closed-loop control.

## Evidence boundary

Static source inspection and an independent Python recurrence can establish structure and simulated
reference behavior. MATLAB syntax execution, MATLAB numerical behavior, Live Editor order, plots,
`uifigure` controls, callbacks, UI cleanup, learner understanding, actuator hardware, servo current,
bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, and production behavior require
separate named evidence; none is implied here.
