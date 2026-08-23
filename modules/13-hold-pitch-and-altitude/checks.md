# P13 checks: Hold Pitch and Altitude

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?

Answer from the observed cascade, not from MATLAB syntax. A complete answer names the altitude and
pitch-loop inputs, follows their effects through pitch and flight path into altitude, identifies the
gain/authority tradeoffs, and diagnoses a reversed altitude/Down feedback sign from growing error.

## What to observe

Start with `model(0.004,2.4,1)`. The fixed-speed reduced-order cascade uses:

```text
e_h       = h_command-h
theta_c   = sat(K_h e_h)
u_theta   = sat(K_p(theta_c-theta)+K_ff theta_c-K_q q)
gamma_dot = (theta-gamma)/tau_gamma
h_dot     = V sin(gamma)
```

Before the command, the complete model is exactly at trim. At the `+30 m` command, pitch command and
equivalent pitch-control command move before pitch; pitch moves before flight-path angle;
flight-path angle then
creates climb rate and altitude change. `Down=-h` must hold at every sample.

## Controlled levers

1. Reset pitch natural frequency to `2.4 rad/s` and sign to `+1`, then sweep altitude gain through
   `[0 0.002 0.004 0.006 0.008] rad/m`. Error at `5 s` falls as gain rises, while overshoot and
   pitch-command saturation expose the path-lag/authority trade. At zero gain, every state and
   control remains exactly at trim and final altitude error remains `30 m`.
2. Reset altitude gain to `0.004 rad/m` and sign to `+1`, then sweep pitch natural frequency through
   `[1.2 1.8 2.4 3.0 3.6] rad/s`. Pitch at `1.5 s` increases, pitch tracking RMS decreases, and peak
   pitch-control demand increases. Outer gain, command, damping ratio, and path time constant
   remain fixed.

Use the exact interactive reset between sweeps. Changing both gains together does not isolate a
mechanism.

## Deliberately broken case

Run `model(0.004,2.4,-1)`. Only the outer-loop sign changes. Correct and broken states are identical
through the command sample. Afterward, a positive altitude error produces negative pitch command,
altitude decreases, and error grows past `301 m`. The pitch-command envelope bounds command and
descent rate over the retained horizon; it does not bound altitude error or restore stability.

Diagnose positive feedback before tuning. This controller has no integral-of-error state or action,
and the model has no stochastic sensor error, wind, or dynamic actuator, so integrator windup,
noise, gust, and actuator lag are not explanations for this symptom.

## Executable invariants

`run_checks.m` independently requires:

- deterministic equality of repeated baseline calls and fixed 1501-sample/1500-interval resources;
- exact pre-step trim and signed command/control onset;
- independent reconstruction of controller gains and every altitude, pitch, pitch-rate, and
  flight-path recurrence;
- `Down=-h`, `h_dot=V*sin(gamma)`, and exact pitch-command/pitch-control envelopes;
- baseline signed reference values, capture, overshoot, tracking, and final-error bounds;
- the zero-gain open-altitude-loop limit;
- two five-point sweeps that preserve the nonselected lever and change the intended observables;
- exact broken state-history isolation through command onset, opposite command signs, growing-error
  symptom, and continued error growth during the final saturated second;
- rejected below/above-range, nonscalar, complex, `NaN`, and `Inf` inputs;
- deterministic recovery and rollback from rejected and broken calls to a fresh correct baseline;
- eight accepted corners and a capped representative grid with finite, fixed-size histories;
- compatibility with the declared base-MATLAB synchronous interface and no persistent state.

The model creates no background work, external resource, timer, future, or worker, so computational
timeout and cancellation paths are not applicable. The fixed grid and capped test matrix are the
relevant resource bounds. MATLAB runtime execution remains unperformed until run on a named MATLAB
environment.

## Interpretation questions

1. Why does pitch move before altitude, and why must flight-path angle appear between them?
2. What physical/computational quantity does `K_h` convert, and what remains fixed during its sweep?
3. Why can higher outer gain reduce early error yet increase overshoot and saturation?
4. What does higher inner natural frequency improve, and what control-authority cost rises?
5. Why is a finite saturated broken trace still a failed feedback loop?
6. How does `h=-Down` determine the correct sign of the outer-loop correction?
7. Which behaviors are declared teaching approximations rather than aircraft or controller-fidelity
   evidence?

## Teach-back

In two sentences, answer the guiding question. First trace a positive altitude error through the
outer gain, bounded pitch command, inner pitch loop, flight-path lag, climb rate, and altitude.
Second name both gain/authority tradeoffs and explain why a command-minus-measurement Down error used
as altitude error reverses feedback and makes altitude error grow.
