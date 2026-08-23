# P15 lesson: Control Speed with Throttle

## Guiding question

What inputs, observable effects, and failure modes matter when you control Speed with Throttle?

## Compounds on P14

P14 used a cascade:

```text
heading error -> bank command -> inner bank response -> heading rate -> heading
```

It deliberately fixed true airspeed at `60 m/s`. P15 carries the same cause-and-effect discipline
into the longitudinal path and makes speed dynamic:

```text
speed error -> thrust command -> delivered throttle -> net forward force -> speed
```

The connection is conceptual rather than current API compatibility. P15 does not accept a P14
heading, bank, history, or controller output, and it does not run heading hold simultaneously.
P04's drag balance, P10's requested-versus-delivered actuator distinction, and P12's
force/energy conventions are also conceptual inputs rather than adapters.

## Begin from forward force, not from a throttle plot

Along the declared straight and level flight path, positive force and speed point forward:

```text
m V_dot = T-D
```

If delivered thrust equals drag, acceleration is zero and speed remains constant. More throttle is
not itself speed; it first changes delivered thrust, then net force, then acceleration, and only
then accumulated speed. That ordering is the first baseline observation.

P15 holds mass, density, lift equal to weight, and aerodynamic coefficients fixed. Substituting
`q=0.5 rho V^2` and `CL=W/(q S)` into `D=q S(CD0+k CL^2)` makes both drag mechanisms visible:

```text
D_parasite = 0.5 rho S CD0 V^2
D_induced  = 2 k W^2/(rho S V^2)
D          = D_parasite + D_induced
```

Parasite drag rises with `V^2`. In this constant-lift approximation, induced drag falls with
`1/V^2`. This is P04's level-flight relation carried into a dynamic speed exercise. It is not a
post-stall model, an aerodynamic database, or a substitute for coupled lift and flight-path
dynamics.

## Speed error becomes a bounded thrust request

The controller uses command minus measurement:

```text
e_V       = V_command-V
a_request = s K_V e_V
T_raw     = D(V)+m a_request
T_command = sat(T_raw,0,T_max)
```

`K_V` has units `1/s`. Multiplying `K_V` by speed error gives desired acceleration, and
multiplying by mass gives corrective force. With correct feedback `s=+1`, a positive speed error
adds thrust. With the deliberately broken `s=-1`, the same error removes thrust.

The visible `D(V)` term is exact model-based drag feedforward. It makes zero error correspond to
the thrust needed for this declared trim. Real aircraft do not know drag exactly across their
envelope, so this is a transparent teaching boundary rather than an identified feedforward law,
gain schedule, or robustness result.

The thrust command is limited to `0 <= T_command <= 4000 N`. Saturation prevents negative thrust
or a request beyond the teaching cap. It does not guarantee that the speed command is achievable,
repair a wrong feedback sign, or establish envelope protection.

## Requested throttle is not delivered throttle

The normalized throttle command and delivered throttle obey:

```text
delta_command = T_command/T_max
delta_dot     = (delta_command-delta)/tau_T
T             = T_max delta
```

Both throttle quantities are fractions in `[0,1]`. Throttle rate has units `1/s`. At a command
step, requested throttle can change immediately, but delivered throttle is a state and cannot
jump. `tau_T` is an ideal first-order teaching parameter, not a propeller map, jet spool model,
fuel-flow command, P10 adapter, or identified engine time constant.

Finally:

```text
V_dot = (T_max delta-D(V))/m
```

The fixed-step recurrence uses only sample-`k` values for both next states. This keeps the
request, delivery, force, acceleration, and speed sequence inspectable.

## Baseline: read the transition in order

Speed begins at `60 m/s`. The initial `826.952 N` drag sets delivered throttle to `0.206738`,
so thrust-minus-drag and acceleration are exactly zero before the command. At `1 s` the command
changes to `70 m/s` under `K_V=0.15 1/s`, `tau_T=0.8 s`, and `s=+1`.

At the command sample:

- proper and controller-used error become `+10 m/s`;
- requested acceleration becomes `+1.5 m/s^2`;
- raw and bounded thrust command become about `2626.952 N`;
- commanded throttle becomes about `0.656738`;
- delivered throttle remains `0.206738`;
- delivered thrust still equals drag, so net force and acceleration remain zero;
- speed remains `60 m/s`.

At the next updates, delivered throttle rises. By `1.5 s` it is about `0.416366` and
acceleration is about `0.697472 m/s^2`. Speed reaches about `64.071756 m/s` at `5 s` and
`67.490612 m/s` at `10 s`. Correct error first enters the `1 m/s` 90%-capture band
`14.34 s` after the command, then enters and remains in the `0.2 m/s` settling band
`23.68 s` after command. Final speed is about `69.920075 m/s`.

These are deterministic reduced-order references, not performance requirements for an aircraft or
engine.

## Lever 1: speed-feedback gain

Reset `tau_T=0.8 s` and correct sign, then sweep
`K_V=[0,0.075,0.15,0.225,0.3] 1/s`.

- `K_V=0` is the exact feedback-open force-trim limit. The displayed command changes, but current
  drag feedforward remains equal to delivered thrust, throttle does not move, and speed remains
  `60 m/s`. Final speed error remains `10 m/s`.
- Increasing gain adds more corrective newtons for the same speed error. Speed at `5 s` rises and
  error at `10 s` falls monotonically across the retained sweep.
- Peak delivered throttle rises with gain. At `K_V=0.3 1/s`, the initial raw request exceeds
  `4000 N` and the thrust command saturates.

Mechanism first: gain changes the requested corrective acceleration and force. It does not change
throttle lag, mass, drag coefficients, density, command, thrust cap, or feedback sign. Higher gain
is not free performance because throttle authority is finite.

## Lever 2: throttle time constant

Reset `K_V=0.15 1/s` and correct sign, then sweep
`tau_T=[0.2,0.5,0.8,1.1,1.4] s`.

Every case uses the same controller law, gain, drag equation, limits, and initial state, so the
command-onset request is identical. A smaller time constant moves delivered throttle closer to that
onset request, gives more speed by `2 s`, and lowers request-delivery tracking RMS. Once delivered
throttle makes the speed histories diverge, subsequent requests differ through feedback. The cost
of faster delivery is a larger peak normalized throttle rate: about `2.25 1/s` at `0.2 s` versus
about `0.3214 1/s` at `1.4 s`.

Mechanism first: the time constant directly changes actuator delivery, while the controller law,
gain, drag equation, feedback sign, and maximum thrust stay fixed. The retained lesson compares
early speed, tracking RMS, and throttle-rate demand. It does not claim an unchanged closed-loop
request history, a monotonic final-error result, engine feasibility,
bandwidth margin, or improved closed-loop robustness.

## Deliberately broken reversed feedback

The broken call preserves the grid, constants, command, gain, lag, initial speed, initial throttle,
drag, saturation, actuator, and plant equations. It changes only:

```text
correct: speedErrorUsed = +(V_command-V)
broken:  speedErrorUsed = -(V_command-V)
```

Correct and broken state histories are identical through command onset because neither state can
jump. Their commands immediately diverge: correct mode asks for about `2626.952 N`, while broken
mode's raw request is about `-973.048 N` and saturates to idle.

Delivered throttle then decays, drag exceeds thrust, and speed falls. Because the proper error is
`V_command-V`, falling `V` makes that error larger. The wrong sign therefore creates positive
feedback even though the command is bounded. By `30 s`, speed is about `40.9897 m/s` and proper
error is about `29.0103 m/s`.

The final retained second is not recovery: throttle command stays at idle, every speed step is
negative, proper error grows by about `0.722 m/s`, and terminal acceleration is about
`-0.7271 m/s^2`. The entire fixed trace remains above the declared `37.55 m/s` stall boundary.
This supports a fixed-horizon diagnosis, not an infinite-horizon stability or post-stall claim.

The failure is not throttle lag—the correct case has the same lag. It is not integrator windup,
because there is no integral-of-error state. It is not a sensor, wind, or propulsion-map failure,
because those mechanisms do not exist here. Diagnose feedback sign before tuning gain or lag.

## Numerical and limiting invariants

- Every call retains 1501 samples, 1500 updates, and the same `0:0.02:30 s` grid.
- Command, speed, throttle, forces, and derivatives remain at exact trim before the step.
- Parasite drag, induced drag, total drag, corrective acceleration, unclamped/bounded thrust,
  throttle rate, delivered thrust, net force, and acceleration can be independently reconstructed.
- Every next speed and delivered-throttle sample follows the sample-`k` forward-Euler recurrence.
- Commanded and delivered throttle remain within `[0,1]` for every accepted input.
- Every accepted corner and broken case remains finite, positive-speed, and above the declared
  stall boundary over the retained horizon.
- Zero speed gain is the exact feedback-open force-trim limit.
- Each five-point sweep changes one public lever while fixing sign, grid, command, constants, and
  the other lever.
- Correct and broken states match through command onset; only the selected feedback sign changes.
- Rejected inputs leave no persistent state; valid calls after rejection or broken mode reproduce
  baseline exactly.
- Eight accepted corners and a capped 18-case representative grid retain fixed work and histories.

## Common misconceptions

- Throttle is not speed. Throttle changes thrust; net force changes acceleration; acceleration
  accumulates into speed.
- Requested throttle is not delivered throttle. The lag state is visible.
- A positive speed error must add forward force under the declared sign convention.
- Feedforward drag balance is not feedback and does not make the controller robust to model error.
- Zero gain does not command idle; it leaves only declared drag-balancing feedforward.
- More gain is not free because the thrust command saturates.
- A smaller time constant is not free because throttle-rate demand rises.
- Normalized throttle is not thrust in newtons and is not fuel flow.
- True airspeed is not ground speed; wind is absent.
- Holding level lift algebraically is not simultaneous P13 altitude control.
- The wrong-sign case is positive feedback, not windup.
- Staying above a modeled stall boundary is not flight-envelope or aircraft-safety evidence.
- P15 adds no sensor, estimator, wind, gust, engine deck, gain schedule, delay, full 6-DOF coupling,
  fault tolerance, hardware timing, or HIL behavior.

## Evidence boundary

Static source inspection and an independent standard-library Python equation oracle can establish
structure and simulated reference behavior. MATLAB syntax execution, MATLAB numerical behavior,
Live Editor order, figures, `uifigure` controls, callbacks, learner understanding, controller,
aircraft or propulsion fidelity, bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment,
staging, and production behavior require separate named evidence and are not implied here.
