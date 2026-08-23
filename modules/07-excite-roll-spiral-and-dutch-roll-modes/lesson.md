# P07 lesson: Excite Roll, Spiral, and Dutch-Roll Modes

## Guiding question

What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?

## Compounds on P06

P06 separated short period from phugoid by pairing each excitation with its dominant observables,
then comparing seconds-scale and tens-of-seconds-scale responses. It also separated restoring
stiffness from damping. P07 keeps that observation method and changes axes: roll rate `p`, bank
angle `phi`, sideslip `beta`, yaw rate `r`, and heading `psi` are lateral-directional quantities.
The same rule still applies: mode names come from response shape and time scale, not array order or
plot color.

The reference condition carries forward P06's `V0 = 60 m/s`,
`qbar = 1325.00798531847 Pa`, and `S = 16.2 m^2`. P07 declares a notional `10.9 m`
wingspan, roll and yaw inertias, control-moment gains, modal decay rates, and Dutch-roll frequency.
Those declared values make the mechanisms visible; they are not identified stability derivatives.

## Sign conventions before motion

The body axes are right-handed: `x` forward, `y` right, and `z` down. Positive `p` and `phi` mean
right-wing-down. Positive `r` and `psi` mean nose or heading right. `beta = atan2(v,u)`, so positive
`beta` means air-relative velocity toward body right.

The aileron and rudder inputs are operational commands: positive aileron creates a positive roll
moment and positive rudder creates a positive yaw moment. No trailing-edge direction is claimed,
because surface-sign conventions vary. With inertial flight-path direction nearly frozen, yawing
the nose right makes body-frame lateral velocity initially negative, so the teaching closure is
`beta_dot ~= -r`. Sideforce, bank/gravity coupling, and flight-path rotation make that relationship
only an approximation in a complete aircraft model.

## Three modes, three signatures

### Roll subsidence

The aileron pulse is collapsed to an angular impulse:

```text
L_delta_a = qbar S b C_l_delta_a
p(0+)     = L_delta_a delta_a Delta t / I_x
p_dot     = -lambda_R p
phi_R_dot = p
```

Thus `p = p(0+) exp(-lambda_R t)`, with time constant `1/lambda_R`. Bank angle is the
integral of roll rate, so the isolated roll component approaches `p(0+)/lambda_R` rather than
returning to zero. Damping a body rate is not the same as commanding wings level.

The baseline `+2 deg` aileron command creates `p(0+) = 2.69533 deg/s`. At
`lambda_R = 2.5 1/s`, the time constant is `0.4 s`, the 2% settling time is
`1.56481 s`, and the roll-mode bank increment approaches `1.07813 deg`.

### Spiral

An initial `+5 deg` right-wing-down bank exposes the slow stable baseline:

```text
phi_S_dot = -lambda_S phi_S
phi_S     = phi_S(0) exp(-lambda_S t)
psi_dot   ~= (g/V0) phi_S
```

At `lambda_S = 0.025 1/s`, the bank half-life is `27.7259 s`; after `120 s`,
`phi_S = 0.248935 deg`. The heading view integrates the small-angle coordinated-turn proxy from
P01 and reaches `31.0614 deg`. That proxy is a useful observable, not the spiral eigenvector and not
a claim of a coordinated full-aircraft trajectory. At the neutral limit `lambda_S = 0`, bank
persists and the proxy heading grows linearly.

### Dutch roll

The rudder pulse is also collapsed to an angular impulse:

```text
N_delta_r = qbar S b C_n_delta_r
r(0+)     = N_delta_r delta_r Delta t / I_z
beta_dot  = -r
r_dot     = omega_D^2 beta - 2 zeta_D omega_D r
```

Combining the two states gives an underdamped sideslip oscillator. The baseline `+3 deg` rudder
command creates `r(0+) = 3.15859 deg/s`. With `omega_D = 1.15 rad/s` and
`zeta_D = 0.18`, the damped period is `5.55436 s`, the envelope keeps
`0.316715` of its amplitude per cycle, and sampled peak sideslip is `2.12919 deg`.
The model's quadratic modal energy is
`E = 0.5 (r^2 + omega_D^2 beta^2)`, so
`E_dot = -2 zeta_D omega_D r^2`. This is a diagnostic state norm with units
`rad^2/s^2`, not physical energy in joules.

The baseline time-scale ratios are `T_D/tau_R = 13.8859` and
`tau_S/T_D = 7.20155`: roll rate fades first, Dutch roll oscillates over several seconds, and
spiral bank changes over tens of seconds.

## Baseline, then one lever

The deterministic baseline uses aileron pulse `+2 deg`, bank release `+5 deg`, rudder pulse
`+3 deg`, roll decay rate `2.5 1/s`, spiral decay rate `0.025 1/s`, and Dutch-roll
damping ratio `0.18`. Inspect roll rate before its integrated bank change, sideslip before yaw
rate, and spiral bank before heading.

First sweep `lambda_R` through `1.0, 1.5, 2.5, 3.5, 5.0 1/s`. The pulse moment and
inertia keep `p(0+)` fixed. Larger `lambda_R` removes that rate sooner and produces less integrated
bank change. Every spiral and Dutch-roll vector remains identical.

Reset `lambda_R = 2.5 1/s`, then sweep `zeta_D` through `0, 0.08, 0.18, 0.30, 0.45`.
The sweep plots amplitude-envelope retention, not energy retention: after one damped period the
energy at the same phase retains the square of the plotted amplitude ratio. Zero damping preserves
both. Positive damping removes energy whenever yaw rate is nonzero; the damped period changes
slightly while the declared natural frequency stays fixed. Every roll and spiral vector remains
identical.

## Inputs excite; modal parameters shape

Setting the aileron pulse to zero removes only roll rate and its bank integral. Setting the bank
release to zero removes only spiral bank, spiral-mode roll rate, and heading change. Setting the
rudder pulse to zero removes only sideslip, yaw rate, and Dutch-roll energy. Reversing one input
reverses its owned linear states; reversing the rudder preserves quadratic modal energy.

Real aileron and rudder inputs generally project into several coupled modes. Direct bank release is
an initial condition, not a surface pulse. The deliberate decoupling lets the learner identify
modal signatures before P08 connects derivatives to coupled motion.

## Deliberately broken spiral sign

The stable baseline uses `exp(-lambda_S t)`. The broken case substitutes
`exp(+lambda_S t)` while keeping the initial bank and rate magnitude. At first the change is slow,
but `5 deg` grows to `100.428 deg` at `120 s` and crosses the declared `15 deg`
small-disturbance bank limit at the first sampled time `44 s`.

A weakly unstable spiral can be physically real. The failure here is assuming the stable baseline
sign while implementing the opposite sign, then treating the out-of-domain continuation as a valid
large-bank prediction. The trace after `15 deg` is a recognizable failure symptom only.

At the other extreme, the model accepts finite decay rates from `0` through `0.05 1/s`. It
evaluates the near-neutral heading integral with a cancellation-safe series or `expm1`. If the
implied positive time constant is larger than finite binary64 can represent, histories remain finite
while the time constant is reported as `Inf` with an explicit numeric-range label.

## Scope and common misconceptions

- Roll subsidence damps roll rate; it does not by itself level the wings.
- A stable spiral returns bank toward zero but can leave a finite heading change.
- Dutch roll is identified here by sideslip/yaw oscillation; a real Dutch-roll eigenvector normally also contains roll participation.
- Positive `beta` and positive rudder depend on declared conventions; never infer signs from a generic sketch.
- `beta_dot ~= -r` assumes nearly frozen flight-path direction and omits sideforce and gravity coupling.
- `psi_dot ~= g phi/V0` is a small-angle coordinated-turn observable, not a full spiral-mode derivation.
- A slow spiral divergence may look trimmed over a short window; observe long enough to identify its sign.
- Decoupled analytic modes cannot establish handling qualities, stability derivatives, or vehicle fidelity.

## Dependency and evidence boundary

The implementation uses base MATLAB element-wise arithmetic, plots, and `uifigure` controls. It
does not call Control System Toolbox, Simulink, a numerical ODE or eigen solver, external I/O, or
random state. Static source checks and an independent Python equation oracle can verify structure
and the declared equations. They do not prove MATLAB execution, MATLAB numerical fidelity, Live
Editor order, graphics, callbacks, learner understanding, bench, HIL, field, or production behavior.
