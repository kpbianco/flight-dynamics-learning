# P10 checks: Model Actuator Dynamics and Limits

## Guiding question

What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?

Ask and answer one item at a time.

## Observation check

At the first `+25 deg` request, point to requested command, feasible `+15 deg` command, raw lag rate,
delivered rate, and delivered deflection in that order. Why does the surface move at `45 deg/s`
before bending into a first-order response?

## P09 boundary check

Trace the command through position limiting, lag, rate limiting, state update, and the declared
`80 N*m/deg` gain. Which signal is a flight-computer request, which is actuator state, and which
conceptual body-y moment would a later rigid-body adapter use? Why can this ledger not be passed
directly to P09's current scalar pulse-scale API, and why is the gain an interface demonstration
rather than aerodynamic fidelity?

## First-lever check

Reset the rate limit to `45 deg/s`, then increase `tau`. Predict the position envelope, delivered
peak rate, time to 90% of the feasible `+15 deg` target, and feasible-command RMS error. Why does a
larger `tau` lower raw demand only for the same remaining error, and why can rate-limited duration
decrease while tracking error increases across evolving trajectories?

## Second-lever check

Reset `tau` to `0.18 s`, then increase rate authority. Predict the reversal slope, zero-crossing
delay, and feasible-command RMS error. At what point would still more rate authority stop changing
the trace?

## Limiting-case and interpretation checks

- Why are position magnitude in `deg` and motion rate in `deg/s` independent limits?
- Why is a first-order time constant not a pure transport delay?
- Why does a zero position error give an exact zero rate and zero state update?
- With `tau=0.50 s` and a `120 deg/s` rate limit, why is the rate clip inactive for this schedule?
- During an inactive-rate interval, derive
  `delta_(k+1)=(1-dt/tau) delta_k+(dt/tau) delta_command_limited`.
- Why does `dt/tau <= 0.2` make each accepted complete update a partial move toward its limited
  target? Why retain a defensive post-update state clip even though the candidate-state checks prove
  it inactive on this declared grid and domain?
- Why does a smaller `tau` not create more motion while the rate stop is active?
- Why is requested moment different from feasible and delivered moment?
- How does P10 extend P09 without silently implementing aerodynamics, P11 sensors, or P13 control?

## Broken-case check

The deliberately broken trajectory omits target and defensive state enforcement of the one position
envelope while preserving the command schedule, first-order lag, `45 deg/s` rate stop, time grid,
and moment gain. Explain why a smooth, finite, rate-bounded trace can still be physically impossible.
Why must the infeasible-request ledger remain true even when enforcement is disabled? Distinguish the
`1998.952 N*m` total broken peak from its `798.952 N*m` excess beyond feasible authority, and explain
how the excess would contaminate a later rigid-body integration.

## Range, malformed-input, recovery, isolation, and resource check

`run_checks.m` rejects time constants outside `[0.05,0.50] s`, rate limits outside
`[20,120] deg/s`, nonscalar, complex, `NaN`, and `Inf` inputs. It then repeats the valid baseline to
show that rejected calls do not poison recovery. The two sweeps hold their non-lever inputs and
command histories exactly fixed, which checks isolation.
The UI reset button restores the exact baseline rather than relying on approximate slider placement.

Each trajectory is synchronous and fixed at 501 samples and 500 updates. A public model call performs
one complete and one broken trajectory; the focused corner/grid checks remain explicitly capped.
There is no file, network, device, process, timer, future, or parallel work to time out or cancel, so
timeout and cancellation are deliberately not runtime semantics of this API. The learner CLI keeps
its independent ten-second subprocess timeout in isolated fixtures.

Base MATLAB arithmetic and graphics are the intended compatibility boundary. MATLAB release,
graphics, callbacks, accessibility, Octave, Windows, and PowerShell behavior require execution in
those named environments. No data or learner-state migration occurs. Rollback removes only P10-owned
implementation artifacts and restores P10 lifecycle fields after coordinating any later dependent
frontier; no backup or state recovery operation is needed.

## Executable checks

`run_checks.m` covers deterministic shape and finite bounds; independent command reconstruction;
every complete and broken recurrence update; position/rate invariants; recognizable baseline
metrics; the rate-to-lag regime transition; the inactive-rate limiting case; two isolated sweeps;
the defensive state-clip limiting case; the omitted-position-envelope symptom and first divergence;
malformed-input rejection; recovery; and capped resource work.

Run from the repository root:

```matlab
run_module_checks("P10")
```

## Teach-back

In two sentences, answer the guiding question. Sentence one must trace the inputs and mechanisms
from requested deflection to the conceptual body-y moment ledger, naming units and the difference
between lag, rate, and position limits. Sentence two must diagnose the broken position-envelope case from
its visible deflection, retained infeasible-request ledger, and invented-authority excess without
referring to MATLAB syntax.
