# P10 — Model Actuator Dynamics and Limits

**Track:** Flight Dynamics and Aerospace GNC

**Phase 3:** Six-degree-of-freedom simulation

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?

## Physical mental model

P09 accepted body force and moment as though they arrived immediately. P10 inserts a generic
single-axis control-surface actuator before the pitch-moment input:

```text
command (deg)
   -> position-limited target (+/-15 deg)
   -> first-order lag (tau)
   -> rate-limited motion (+/-rate_limit deg/s)
   -> delivered deflection delta (deg)
   -> declared conceptual body-y moment M_y = 80 delta (N*m)
```

The state cannot jump. Position limiting bounds how far it may travel, rate limiting bounds its
slope, and the time constant shapes unsaturated error closure. On the declared grid and parameter
domain, every complete update moves toward a position-limited target, so the post-update state clip
is a defensive guard rather than an active baseline mechanism. Requested command, feasible command,
and delivered deflection are deliberately plotted as separate signals.

The moment ledger has the same units and body-y meaning as P09's internal applied pitch moment, but
the modules are not directly composable: P09's public `model(forcePulseScale,momentPulseScale)` API
does not accept an arbitrary moment history, the grids differ, and P10 supplies no adapter.

## Deterministic experiment

The fixed schedule holds `0 deg` to `0.5 s`, requests `+25 deg` to `2.0 s`, reverses to `-25 deg`
to `3.5 s`, and finishes at `+5 deg` through `5.0 s`. The baseline actuator uses `tau=0.18 s`, a
`45 deg/s` symmetric rate limit, a `+/-15 deg` hard stop, and a visible `0.01 s` explicit recurrence.
Each complete or broken trajectory retains exactly 501 samples and 500 bounded updates.

Independent simulated equation evaluation gives a `0.45 s` response to 90% of the feasible
`+15 deg` target, a reversal zero crossing near `0.34 s`, `0.92 s` of rate limiting,
feasible-command RMS error near `7.70015 deg`, and peak delivered pitch moment near
`1199.706 N*m`. These are deterministic teaching references, not MATLAB-runtime or
actuator-identification evidence.

## Two independent levers

1. Reset the rate limit to `45 deg/s`, then sweep time constant through
   `[0.08,0.12,0.18,0.28,0.40] s`. For the same remaining error, a larger time constant lowers raw
   unsaturated rate demand. Across the evolving trajectories it delays the feasible-target response
   and increases tracking error without moving either limit; pointwise raw-rate ordering is not
   assumed because each trajectory retains a different error.
2. Reset the time constant to `0.18 s`, then sweep rate authority through
   `[20,30,45,60,80] deg/s`. More rate authority steepens rate-limited segments, shortens response,
   and lowers feasible-command tracking error without changing lag or the hard stop.

Use the interactive reset button to restore exactly `tau=0.18 s` and `45 deg/s` between levers.

With `tau=0.50 s` and `120 deg/s`, the rate stop never activates for the declared schedule. That
limiting case reduces to the visible first-order recurrence with a position-limited target. The
defensive post-update state clip remains inactive throughout the accepted domain.

## Deliberately broken position envelope

The broken comparison keeps the same command, lag, rate limit, grid, and moment gain but omits both
enforcement points of the one position-envelope assumption: target saturation and the defensive
post-update guard. It remains smooth and rate bounded while reaching about `24.987 deg`, nearly
`9.987 deg` beyond the declared position envelope. The mapped peak moment grows to about
`1998.952 N*m`, or about `798.952 N*m` beyond the maximum feasible magnitude, creating impossible
authority that would contaminate a later rigid-body integration if the conceptual boundary were
trusted blindly. The infeasible-request ledger remains true in the broken output so the diagnostic
cannot disappear merely because enforcement was removed.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P10")
run_module_checks("P10")
```

The implementation uses base MATLAB arithmetic, graphics, `uifigure`, and a fixed synchronous loop.
It does not use Control System Toolbox, Simulink, an ODE solver, random data, file or network I/O,
timers, or parallel workers. There is no background task to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and concise command-to-moment narrative.
- `model.m` — guarded deterministic lag, rate stop, position envelope, metrics, and broken comparison.
- `experiment.m` — baseline views, two isolated sweeps, limiting case, and broken position envelope.
- `interactive.m` — time-constant/rate controls, exact reset, and immediate signal/envelope views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, mechanism explanations, and observation order.
- `checks.md` and `run_checks.m` — interpretation questions and independent numerical invariants.

Static inspection and an independent Python recurrence can validate source structure and simulated
reference results without MATLAB. They do not establish MATLAB execution, Live Editor order,
graphics, UI callbacks, MATLAB numerical fidelity, instructional effectiveness, actuator hardware,
bench, HIL, field, release, deployment, or production evidence.
