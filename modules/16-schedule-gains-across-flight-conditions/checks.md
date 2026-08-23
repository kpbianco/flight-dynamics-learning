# P16 checks: Schedule Gains Across Flight Conditions

## Guiding question

What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?

Answer from the observed condition–lookup–controller–plant chain, not from MATLAB syntax. A complete
answer names true airspeed and density, dynamic pressure, the actual-versus-lookup condition,
interpolated angle and rate gains, equivalent aileron limits, roll response, and the risk of a wrong
or out-of-envelope scheduling variable.

## What to observe

Start with `model(60,0.736115547399152,1)`:

```text
qbar       = 0.5 rho V^2
b          = b_ref qbar/qbar_ref
delta_a    = sat(K_phi*(phi_command-phi)-K_p*p,+/-15 deg)
phi_dot    = p
p_dot      = b delta_a
```

The center knot supplies `K_phi=0.48 rad/rad` and `K_p=0.32 s`. At command onset, equivalent
aileron becomes `4.8 deg`, acceleration changes, then roll rate and angle move through the explicit
state updates. Scheduled and fixed-reference modes are identical only at this reference condition.

## Controlled levers

1. Reset density to `rho_ref` and mode to dynamic-pressure scheduling, then sweep true airspeed
   through `[45 52.5 60 67.5 72] m/s`. Scheduled settling remains within `1.55–1.56 s`, while
   peak equivalent aileron falls from `8.8` to about `3.3536 deg`. Fixed gains expose changing
   effective poles; do not confuse a condition-dependent speed-up with preserved response.
2. Reset airspeed to `60 m/s` and sweep density through
   `[0.5 0.75 1 1.25 1.5]*rho_ref`. These exact knots produce equal scheduled roll histories,
   while peak equivalent aileron falls from `9.6` to `3.2 deg`.

Use the exact interactive reset between sweeps. Changing airspeed and density together usually
confounds their independent contributions, except for the deliberate equal-dynamic-pressure test.

## Deliberately broken case

Run the correct and broken calls at:

```matlab
pairedDensity=0.736115547399152*(60/75)^2;
correct=model(75,pairedDensity,1);
broken=model(75,pairedDensity,-1);
```

The actual dynamic pressure equals the reference condition in both calls. Correct mode reproduces
the reference histories. Broken mode omits density, computes raw lookup ratio `1.5625`, and holds it
to the `1.5` endpoint. It chooses gains that are too small, settles near `3.06 s`, and overshoots by
about `0.693 deg` rather than `0.162 deg`.

Only schedule selection changes. The actual plant pressure, command, initial state, grid, and
control limit remain fixed. Clamping prevents table extrapolation but does not restore the correct
gains. The symptom is scheduling-variable mismatch, not feedback reversal, actuator lag, or
integrator windup.

## Executable invariants

`run_checks.m` independently requires:

- deterministic repeated baseline equality and fixed 801-sample/800-interval resources;
- exact pre-step rest, command-onset ordering, and signed baseline references;
- independent reconstruction of dynamic pressure, actual control effectiveness, raw and clamped
  lookup ratios, bracket indices, interpolation weight, both gain tables, a fractional midpoint,
  interpolated gains, used-lookup table error, actual-condition gain mismatch,
  controller output, saturation, acceleration, and every state recurrence;
- exact gain/pole closure and coincident trajectories at all five density knots;
- reference scheduled/fixed equality and equal-dynamic-pressure condition invariance;
- two five-point sweeps that preserve the nonselected condition and expose intended response and
  control-demand changes, with every reported peak reconstructed from its command history;
- lower and upper endpoint clamping without extrapolation or alteration of the actual plant;
- isolated broken lookup, onset command divergence, slower/less-damped response, active clamp,
  deterministic rollback, and baseline recovery;
- rejected below/above-range, nonscalar, complex, `NaN`, `Inf`, and invalid-mode inputs;
- accepted corners and a capped representative grid with finite, fixed-size histories and bounded
  equivalent aileron command;
- compatibility with the declared base-MATLAB synchronous stateless interface.

The model creates no background task, external resource, timer, future, worker, or callback loop, so
computational timeout and cancellation paths are not applicable. The fixed grid, bounded public
inputs, five table knots, endpoint hold, and capped case matrix are the applicable resource bounds.
It changes no learner data, score, database, service, schema, migration, or backup/restore path.

MATLAB runtime execution remains unperformed until these checks run on a named MATLAB environment.

## Interpretation questions

1. Why must density accompany true airspeed when dynamic pressure is the scheduling variable?
2. Which quantity changes the actual plant, and which quantity selects table gains?
3. Why do `K_phi` and `K_p` decrease as dynamic pressure rises in this declared plant?
4. Why can scheduled roll histories overlay while equivalent aileron histories differ?
5. What does fixed-gain comparison reveal away from the reference knot?
6. Why are scheduled and fixed modes exactly identical at `qbar/qbar_ref=1`?
7. What approximation is introduced between the exact gain knots?
8. Why is endpoint clamping a visible envelope warning rather than a safety guarantee?
9. How does the equal-dynamic-pressure pair isolate the broken scheduling variable?
10. Which omitted condition variables and dynamics prevent aircraft, robustness, or certification
    claims?

## Teach-back

In two sentences, answer the guiding question. First trace true airspeed and density through dynamic
pressure, actual control effectiveness, bounded table interpolation, angle/rate feedback, equivalent
aileron, and roll response. Second explain how the two condition sweeps trade gain/control demand for
response consistency, then diagnose why a true-airspeed-only lookup can select the wrong gains even
when actual dynamic pressure is unchanged.
