# P15 checks: Control Speed with Throttle

## Guiding question

What inputs, observable effects, and failure modes matter when you control Speed with Throttle?

Answer from the observed force-and-actuator chain, not from MATLAB syntax. A complete answer names
speed error, gain, throttle response, drag feedforward, thrust limits, and feedback sign; follows
delivered thrust minus drag into acceleration and speed; and diagnoses reversed feedback from idle
throttle plus growing proper error.

## What to observe

Start with `model(0.15,0.8,1)`:

```text
e_V       = V_command-V
D(V)      = 0.5 rho S CD0 V^2 + 2 k W^2/(rho S V^2)
a_request = s K_V e_V
T_command = sat(D(V)+m a_request,0,T_max)
delta_dot = (T_command/T_max-delta)/tau_T
V_dot     = (T_max delta-D(V))/m
```

Before the command, delivered thrust equals `826.952 N` drag, throttle is `0.206738`, and every
state and derivative remains at trim. At the `60` to `70 m/s` command, error and requested
throttle move first. Delivered throttle, thrust, acceleration, and speed follow in that order.

## Controlled levers

1. Reset throttle time constant to `0.8 s` and sign to correct, then sweep speed gain through
   `[0 0.075 0.15 0.225 0.3] 1/s`. Speed at `5 s` rises and error at `10 s` falls; peak
   delivered throttle rises, and the highest gain reaches the thrust-command limit. At zero gain,
   throttle and speed stay exactly at initial force trim while final error remains `10 m/s`.
2. Reset speed gain to `0.15 1/s` and sign to correct, then sweep throttle time constant through
   `[0.2 0.5 0.8 1.1 1.4] s`. A smaller time constant gives more speed at `2 s` and less
   request-delivery tracking error, while peak throttle rate rises.

Use the exact interactive reset between sweeps. Changing both levers together does not isolate a
mechanism.

## Deliberately broken case

Run `model(0.15,0.8,-1)`. Only controller feedback sign changes. Correct and broken speed and
delivered throttle remain identical through command onset, but correct mode requests about
`2626.952 N` while broken mode's negative raw request saturates to idle.

The broken response ends near `40.9897 m/s` with about `29.0103 m/s` proper error. Throttle
command is saturated at idle for more than 96% of retained samples, yet the entire trace stays
above the declared `37.55 m/s` stall boundary.

During the final retained second, commanded throttle remains idle, speed continues falling by more
than `0.72 m/s`, proper error grows by the same amount, and acceleration remains negative. This
establishes continued failure through the observed horizon only. Saturation prevents negative
thrust; it does not make reversed feedback safe.

Diagnose feedback sign before tuning. Correct mode has the same throttle lag, the model has no
integral state, and it contains no wind, sensor, estimator, engine map, or fault injector. The
symptom is positive feedback, not actuator lag, integrator windup, gust response, or sensor noise.

## Executable invariants

`run_checks.m` independently requires:

- deterministic equality of repeated baseline calls and fixed 1501-sample/1500-interval resources;
- exact pre-step drag/thrust trim and signed command-onset ordering;
- direct command → delivered-throttle/acceleration → speed sample ordering, followed by monotonic,
  non-overshooting correct-feedback error contraction;
- independent reconstruction of both drag terms, speed error routing, requested acceleration,
  thrust saturation, normalized throttle, throttle rate, delivered thrust, net force, acceleration,
  and every state recurrence;
- baseline signed reference values, capture, settling, tracking, and above-stall bounds;
- the zero-gain feedback-open force-trim limit;
- two five-point sweeps that preserve the nonselected lever and change intended observables;
- exact broken-state isolation through command onset, idle saturation, falling speed, growing proper
  error, a still-failing final second, and a finite above-stall fixed-horizon endpoint;
- rejected below/above-range, nonscalar, complex, `NaN`, `Inf`, and invalid-sign inputs;
- deterministic recovery and rollback from rejected and broken calls to a fresh correct baseline;
- eight accepted corners and a capped 18-case representative grid with finite, positive,
  fixed-size histories and bounded throttle;
- compatibility with the declared base-MATLAB synchronous stateless interface.

The model creates no background work, external resource, timer, future, worker, or callback loop, so
computational timeout and cancellation paths are not applicable. The fixed grid, lower-bounded
time constant, and capped test matrix are the applicable resource bounds. It changes no learner
data, schema, score, database, service, migration, or backup/restore path.

MATLAB runtime execution remains unperformed until these checks run on a named MATLAB environment.

## Interpretation questions

1. Why does acceleration remain zero at the exact sample when commanded throttle jumps?
2. Which quantity does `K_V` convert, and why does multiplying by mass produce newtons?
3. Why does the controller add current drag to corrective force, and what unrealistic knowledge
   does that feedforward assume?
4. What does higher gain improve, and where does maximum thrust stop helping?
5. What does a smaller throttle time constant improve, and what throttle-rate cost rises?
6. Why is normalized throttle neither speed, thrust in newtons, nor fuel flow?
7. Why is zero gain a force-trim limit rather than idle throttle?
8. Why does reversed feedback make proper error grow even though throttle is bounded?
9. Why is the broken symptom positive feedback rather than integrator windup or actuator lag?
10. Which behaviors are transparent teaching approximations rather than controller, propulsion, or
    aircraft-fidelity evidence?

## Teach-back

In two sentences, answer the guiding question. First trace command-minus-speed error through
drag-plus-corrective-force request, bounded commanded throttle, delivered-throttle lag, and
thrust-minus-drag acceleration into true airspeed. Second name the gain/authority and
lag/throttle-rate tradeoffs, then explain why reversing feedback sign commands idle, slows the
aircraft, and grows proper speed error through the observed horizon.
