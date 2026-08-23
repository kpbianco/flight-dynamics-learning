# P16 lesson: Schedule Gains Across Flight Conditions

## Guiding question

What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?

## Compound P15's airspeed state into a flight condition

P14 held heading with a transparent roll loop, and P15 made true airspeed a state. P16 asks the next
control question: if the plant changes with flight condition, should one fixed set of gains be
expected to preserve the same response?

The connection is conceptual rather than runtime API compatibility. P16 does not consume P15 speed
histories or P14 bank commands, and it does not run speed and heading control simultaneously. It
uses a frozen true airspeed and density for each trace so condition-to-gain cause and effect stays
visible.

## Begin with dynamic pressure, not a lookup table

True airspeed alone is not the aerodynamic condition used by this teaching plant:

```text
qbar = 0.5 rho V^2
```

At the same true airspeed, lower density means lower dynamic pressure. At the same density, dynamic
pressure grows with speed squared. P16 declares that equivalent aileron effectiveness scales in
direct proportion:

```text
sigma    = qbar/qbar_ref
b(sigma) = b_ref sigma
```

This one-factor law exists to expose scheduling mechanics. A real aircraft can also change with
Mach number, Reynolds number, mass, center of gravity, configuration, nonlinear aerodynamics,
actuator limits, structural modes, sensors, and uncertainty. Those mechanisms are absent rather
than hidden in the table.

## Convert desired roll poles into gain knots

The frozen-condition roll channel is:

```text
phi_dot = p
p_dot   = b delta_a
```

The controller is:

```text
delta_a_raw = K_phi (phi_command-phi) - K_p p
delta_a     = sat(delta_a_raw,-15 deg,+15 deg)
```

Without saturation, substitution gives:

```text
phi_ddot + b K_p phi_dot + b K_phi phi = b K_phi phi_command
```

To request target natural frequency `omega_n=2.4 rad/s` and damping ratio `zeta=0.8` at a knot:

```text
K_phi = omega_n^2/b
K_p   = 2 zeta omega_n/b
```

Because `b` grows with dynamic pressure, both gains fall as dynamic pressure rises. This is not
"high speed means less control matters." It means the declared plant creates more roll acceleration
per unit equivalent aileron, so less gain is needed to request the same closed-loop coefficients.

## Make lookup and interpolation inspectable

The table has ordered dynamic-pressure-ratio knots `[0.5,0.75,1,1.25,1.5]`. The model:

1. computes a raw lookup ratio;
2. holds it to the table endpoints if necessary;
3. finds the lower and upper knot;
4. computes a linear interpolation weight;
5. blends both gains with that same weight;
6. exposes every intermediate quantity, including table error at the used lookup and selected-gain
   mismatch against the actual plant condition.

Linear interpolation approximates the ideal reciprocal curve between knots. It is exact at each
knot, but matching endpoints does not prove the approximation, stability, or control authority
everywhere between them. Outside the table, holding an endpoint avoids numerical extrapolation but
does not validate that gain for the actual condition. The visible clamp flag means "outside the
declared schedule," not "safe."

## Baseline: the reference knot is also the fixed-gain limit

At `V=60 m/s` and `rho=0.736115547399152 kg/m^3`, dynamic pressure is about
`1325.007985 Pa`, so `sigma=1`. The schedule selects:

```text
K_phi = 0.48 rad/rad
K_p   = 0.32 s
```

The command changes from `0` to `10 deg` at `0.5 s`. Roll angle and rate are states and cannot jump,
but equivalent aileron command immediately becomes `4.8 deg`. Roll acceleration changes at that
sample; rate changes on the next state update, and angle follows through accumulated rate.

The reference response reaches the 90% band after `1.23 s`, settles after `1.55 s`, and overshoots
by about `0.161546 deg`. Scheduled and fixed modes are exactly identical here because the fixed gains
are the center-knot gains. That equality is a useful limiting case, not evidence that scheduling is
unnecessary away from reference.

## Lever 1: true airspeed at fixed density

Reset density to `rho_ref`, select dynamic-pressure scheduling, and sweep
`V=[45,52.5,60,67.5,72] m/s`.

- Dynamic pressure and plant effectiveness rise with `V^2`.
- Interpolated angle and rate gains fall.
- Scheduled settling stays between `1.55` and `1.56 s` across the retained cases.
- Peak equivalent aileron falls from `8.8` to about `3.3536 deg`.

Now compare fixed reference gains at those same conditions. At `45 m/s`, effective natural frequency
and damping fall to `1.8 rad/s` and `0.6`; settling grows to about `3.29 s` and overshoot to about
`0.982 deg`. Higher dynamic pressure moves the fixed-gain poles the other way. A faster response at
one condition is not free improvement if damping, demand, uncertainty, and unmodeled dynamics also
move.

Mechanism first: speed changes plant effectiveness. The schedule changes gains to compensate. It
does not change density, command, time grid, limit, or the plant's actual dynamic pressure.

## Lever 2: density at fixed true airspeed

Reset airspeed to `60 m/s` and sweep density through
`[0.5,0.75,1,1.25,1.5]*rho_ref`. At reference speed, those density ratios are exactly the five
dynamic-pressure-ratio knots.

Every knot has `b K_phi=2.4^2` and `b K_p=2*0.8*2.4`, so the unsaturated state histories overlay.
The required equivalent aileron does not overlay: its peak falls from `9.6 deg` at the low-pressure
knot to `3.2 deg` at the high-pressure knot.

Mechanism first: density changes the same actual dynamic pressure independently of true airspeed.
The overlay is the visible effect of compensation, while the control trace shows how gain values
changed to create it. Matching response does not invent control authority; if the command limit
were reached, scheduled poles would no longer describe the saturated response.

## Deliberately broken: true airspeed-only scheduling

Consider two equal-dynamic-pressure conditions:

```text
A: V=60 m/s, rho=rho_ref
B: V=75 m/s, rho=rho_ref*(60/75)^2
```

Both have the same actual plant effectiveness. Correct dynamic-pressure scheduling therefore gives
the same lookup gains and numerically equal histories. The broken mode keeps condition B's actual
plant unchanged but looks up gains with `(V/V_ref)^2=1.5625`. It omits density, reaches beyond the
table, clamps to `1.5`, and chooses gains that are too small.

The broken trace settles near `3.06 s` and overshoots by about `0.693 deg`, compared with `1.55 s`
and `0.162 deg` for the correct equal-pressure trace. Its effective frequency and damping fall to
about `1.9596 rad/s` and `0.6532`. Both selected gains are one-third below the ideal values for the
actual reference-strength plant. The wrong schedule input is recognizable because actual dynamic
pressure still reads reference while the lookup ratio says `1.5` and the clamp flag is active.

Do not diagnose this as an actuator failure: the model has no actuator state. Do not diagnose it as
feedback-sign reversal: both gains remain positive. Do not call endpoint clamping a recovery: it
bounds table access but preserves the wrong gains. The violated assumption is that true airspeed
alone identifies aerodynamic control effectiveness.

## Numerical and limiting invariants

- Every call retains 801 samples, 800 updates, and the same `0:0.01:8 s` grid.
- All states and commands are exactly zero before the step.
- Dynamic pressure, plant effectiveness, raw/clamped lookup, bracket, weight, both gains,
  controller output, saturation, acceleration, and every state transition can be independently
  reconstructed.
- At each exact knot, the scheduled unsaturated poles close to `2.4 rad/s` and `0.8`.
- Reference scheduled and fixed histories are exactly identical.
- Different accepted `(V,rho)` pairs with the same dynamic pressure have equal correct-schedule
  histories.
- Endpoint clamping never extrapolates and never changes the actual plant condition.
- Accepted corners and a capped representative grid remain finite, fixed-size, and within the
  equivalent aileron envelope for the retained command.
- Invalid, nonscalar, complex, `NaN`, `Inf`, and invalid-mode calls reject before calculation.
- The stateless model reproduces baseline after rejection or broken-mode execution.

## Common misconceptions

- True airspeed is not dynamic pressure; density matters.
- A gain table is not the plant. Actual and lookup conditions are deliberately separate.
- Identical scheduled responses do not mean the gain values or control demand stayed fixed.
- Fixed gains matching at the reference knot do not make them valid everywhere.
- Linear interpolation is an approximation between knots, not proof between knots.
- Endpoint clamping prevents extrapolation; it does not establish outside-envelope safety.
- More dynamic pressure does not create unlimited safe bandwidth or structural margin.
- Equivalent aileron angle is not a P10 actuator history, hinge moment, or hardware command.
- The broken case is schedule-variable mismatch, not feedback reversal, windup, or actuator lag.
- This frozen reduced-order model adds no Mach, mass, CG, configuration, sensors, uncertainty,
  robustness margin, certification, hardware timing, HIL, or flight behavior.

## Evidence boundary

Static source inspection and an independent standard-library Python equation oracle can establish
structure and simulated reference behavior. MATLAB syntax execution, MATLAB numerical behavior,
Live Editor order, figures, `uifigure` callbacks, learner understanding, control-law or aircraft
fidelity, bench, HIL, field, RT1/RT2, Unreal, signing, release, deployment, staging, and production
behavior require separate named evidence and are not implied here.
