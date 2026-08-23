# P14 checks: Hold Roll and Heading

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Roll and Heading?

Answer from the observed cascade, not from MATLAB syntax. A complete answer names circular heading
error and both loop inputs, follows bank into heading rate, identifies the gain/acceleration/
authority tradeoffs, and diagnoses a raw-angle subtraction from its wrong-way bank command.

## What to observe

Start with `model(0.5,2.4,1)`. The fixed-speed reduced-order cascade uses:

```text
e_psi       = wrap(psi_command-wrap(psi_continuous))
phi_command = sat(K_psi e_psi,+/-12 deg)
phi_dot     = p_phi
p_phi_dot   = omega_phi^2(phi_command-phi)-2 zeta_phi omega_phi p_phi
psi_dot     = g tan(phi)/V
```

Before the command, every signal remains exactly at trim. At the `+170 deg` to `-170 deg` display
change, raw subtraction is `-340 deg` but circular error is `+20 deg`. Bank command and bank
acceleration move before bank; bank moves before heading rate accumulates continuous heading. The
wrapped display later crosses its branch cut while the continuous state remains smooth.

## Controlled levers

1. Reset roll natural frequency to `2.4 rad/s` and mode to wrapped, then sweep heading-to-bank gain
   through `[0 0.25 0.5 0.75 1] rad/rad`. Error at `10 s` and 90% capture time fall as gain rises;
   peak bank rises, and the two largest gains activate the `12 deg` command envelope. At zero gain,
   every state and command remains exactly at trim and final circular error remains `20 deg`.
2. Reset heading gain to `0.5 rad/rad` and mode to wrapped, then sweep roll natural frequency
   through `[1.2 1.8 2.4 3.0 3.6] rad/s`. Bank at `1.5 s` increases, tracking RMS decreases, and
   peak reduced-order bank acceleration increases. Outer gain, command, damping ratio, speed, and
   turn relation remain fixed.

Use the exact interactive reset between sweeps. Changing both levers together does not isolate a
mechanism.

## Deliberately broken case

Run `model(0.5,2.4,0)`. Only the controller's selected error changes: it uses raw displayed-angle
subtraction instead of the independently retained shortest error. Correct and broken states are
identical through command onset. The broken loop immediately commands `-12 deg` bank, travels more
than `110 deg` left during the retained horizon, and finishes with more than `130 deg` shortest
error while actual bank stays below `15 deg`.

During the final retained second, the bank command remains saturated, continuous heading still
moves left by about `1.991 deg`, proper shortest error grows by the same amount, and terminal
heading rate remains about `-1.991 deg/s`. The retained reference checks that saturation has not
arrested the failure within the fixed horizon; it does not establish infinite-horizon behavior.

Diagnose the circular-coordinate failure before tuning. Raw error contracts along the selected
`-340 deg` route, so this is not positive feedback. The model has no integral-of-error state,
dynamic actuator, stochastic sensor error, wind, or coupled yaw mode, so windup, actuator lag,
noise, wind, and Dutch roll are not explanations for this symptom.

## Executable invariants

`run_checks.m` independently requires:

- deterministic equality of repeated baseline calls and fixed 3001-sample/3000-interval resources;
- exact pre-step trim and signed raw/wrapped error, bank-command, and acceleration onset;
- independent reconstruction of half-open heading wrapping, shortest error, saturation, bank
  acceleration, coordinated heading rate, and every state recurrence;
- baseline signed reference values, capture, tracking, envelope, and final-error bounds;
- the zero-gain open-heading-loop limit;
- two five-point sweeps that preserve the nonselected lever and change intended observables;
- exact broken state-history isolation through command onset, opposite bank direction, growing
  independent circular error, long-way travel, a still-failing saturated final second, and finite
  command/state envelopes;
- rejected below/above-range, nonscalar, complex, `NaN`, `Inf`, and invalid-mode inputs;
- deterministic recovery and rollback from rejected and broken calls to a fresh correct baseline;
- eight accepted corners and a capped representative grid with finite fixed-size histories;
- compatibility with the declared base-MATLAB synchronous interface and no persistent state.

The model creates no background work, external resource, timer, future, worker, or callback loop, so
computational timeout and cancellation paths are not applicable. The fixed grid and capped test
matrix are the relevant resource bounds. MATLAB runtime execution remains unperformed until run on
a named MATLAB environment.

## Interpretation questions

1. Why is the `+170 deg` to `-170 deg` command a right turn even though raw subtraction is negative?
2. Why must the controller integrate continuous heading rather than repeatedly integrate a wrapped
   display value?
3. What quantity does `K_psi` convert, and what remains fixed during its sweep?
4. How does higher outer gain reduce early error, and where does bank authority stop helping?
5. What does higher inner natural frequency improve, and what acceleration cost rises?
6. Why does positive right-wing-down bank produce positive heading rate in this declared model?
7. Why is the broken long-way turn not positive feedback or integrator windup?
8. Which behaviors are teaching approximations rather than controller or aircraft-fidelity evidence?

## Teach-back

In two sentences, answer the guiding question. First trace wrapped shortest heading error through
bounded bank command, the inner roll response, and `g tan(phi)/V` into continuous heading. Second
name the outer-gain/authority and inner-speed/acceleration tradeoffs, then explain why raw
subtraction across the `+/-180 deg` branch cut commands a saturated wrong-way turn that persists
through the observed horizon.
