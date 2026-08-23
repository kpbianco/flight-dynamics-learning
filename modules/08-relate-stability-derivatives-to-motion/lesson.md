# P08 lesson: Relate Stability Derivatives to Motion

## Guiding question

What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?

## Compounds on P07

P07 recognized roll subsidence, spiral motion, and Dutch roll from excitation, dominant observables,
time scale, and envelope. Those modes were deliberately separated and their rates were prescribed.
P08 retains P07's signs—positive sideslip is air-relative velocity toward body right, positive roll
rate and bank are right-wing-down, and positive yaw rate is nose-right—but now the states interact.
Derivative contributions create the force and moments from which the motion emerges.

The fixed reference condition carries forward `V0 = 60 m/s`,
`qbar = 1325.00798531847 Pa`, `S = 16.2 m^2`, `b = 10.9 m`,
`I_x = 2500 kg*m^2`, and `I_z = 4000 kg*m^2`. P08 declares
`m = 1200 kg` and a notional lateral coefficient set. These numbers make the chain visible; they
are not identified aircraft derivatives.

## One chain, four coupled states

The perturbation state is `x = [beta, p, r, phi]'`, in radians and radians per second internally.
Rate derivatives are defined against nondimensional body rates:

```text
p_hat = p b/(2 V0)       r_hat = r b/(2 V0)

CY = CY_beta beta + CY_p p_hat + CY_r r_hat
Cl = Cl_beta beta + Cl_p p_hat + Cl_r r_hat
Cn = Cn_beta beta + Cn_p p_hat + Cn_r r_hat

Y = qbar S CY            L = qbar S b Cl       N = qbar S b Cn

beta_dot = Y/(m V0) - r + (g/V0) phi
p_dot    = L/Ix
r_dot    = N/Iz
phi_dot  = p
```

The `-r` term says that a nose-right rotation reduces positive sideslip when the inertial flight
path is nearly frozen. The gravity term says right-wing-down bank accelerates the vehicle toward
body right in this small-disturbance view. Neither is an aerodynamic derivative. `phi_dot = p` is
the small-angle roll kinematic relationship.

The declared fixed derivatives are:

```text
CY_beta = -0.65 /rad      CY_p = -0.03       CY_r = +0.25
Cl_beta = -0.12 /rad                         Cl_r = +0.10
Cn_p    = -0.06                              Cn_r = -0.25
```

`C_l_p` and `C_n_beta` remain learner levers. A positive sideslip therefore begins with left
sideforce, a left-wing-down roll acceleration from negative `C_l_beta`, and a nose-right yaw
acceleration from positive `C_n_beta`. Once `p`, `r`, and `phi` develop, their rate, kinematic, and
gravity terms change all subsequent accelerations. A derivative owns a coefficient contribution;
it does not own a complete mode.

## Dimensional derivative ledger

At the baseline `C_l_p = -0.50` and `C_n_beta = +0.18 /rad`, the rate scale is
`b/(2V0) = 0.0908333333 s`. Multiplying coefficient slopes by the fixed force or moment scale gives:

```text
Y_beta = -13952.3341 N/rad       Y_p =   -58.4925 N/(rad/s)
Y_r    =    487.4373 N/(rad/s)

L_beta = -28076.3892 N*m/rad     L_p = -10626.1334 N*m/(rad/s)
L_r    =   2125.2267 N*m/(rad/s)

N_beta =  42114.5838 N*m/rad     N_p =  -1275.1360 N*m/(rad/s)
N_r    =  -5313.0667 N*m/(rad/s)
```

Dividing those loads by `mV0`, `I_x`, or `I_z` produces the visible constant state matrix:

```text
[-0.193782418  -0.000812396  -0.993230037   0.163444167
 -11.230555682 -4.250453366   0.850090673   0
  10.528645952 -0.318784002  -1.328266677   0
   0             1             0             0]
```

Because the state mixes angles and rates, do not give every matrix entry one blanket unit. Check
each term by the equation and the state column it multiplies.

## Baseline, then one lever

Release `beta(0) = +3 deg` with `p(0) = r(0) = phi(0) = 0`. The initial loads are
`Y = -730.5425 N`, `L = -1470.0763 N*m`, and `N = +2205.1145 N*m`.
They produce `beta_dot = -0.581347 deg/s`, `p_dot = -33.691667 deg/s^2`, and
`r_dot = +31.585938 deg/s^2`. The sampled motion first crosses zero sideslip at `0.54 s`, reaches
peak `|p| = 3.90155 deg/s`, peak `|r| = 7.23028 deg/s`, and peak
`|phi| = 1.64347 deg`.

Early peaks and a finite 25-second trace are not enough to exclude a slow divergence. At every accepted
`C_l_p`/`C_n_beta` corner, the sampled maximum magnitude of each state over `20-25 s` is less than
`99%` of its maximum over `15-20 s`. That is a behavioral check on the declared lesson horizon, not a
general proof that derivative signs alone guarantee coupled stability.

First sweep `C_l_p` through `-0.30, -0.40, -0.50, -0.65, -0.80`, holding every other
input and derivative fixed. Only dimensional `L_p` and state-matrix entry `(2,2)` change directly.
Initial roll acceleration remains fixed because `p(0)=0`. As roll rate develops, more-negative
`C_l_p` opposes it more strongly: peak `|p|` falls from `4.92727` to `2.98320 deg/s`, and peak
`|phi|` falls from `2.29145` to `1.13963 deg`.

Reset `C_l_p = -0.50`, then sweep `C_n_beta` through `0, 0.06, 0.12, 0.18, 0.24 /rad`.
Only dimensional `N_beta` and matrix entry `(3,1)` change directly. Initial yaw acceleration rises
linearly from `0` to `42.11458 deg/s^2`, the first sideslip zero moves from `1.76` to `0.46 s`, and
peak yaw rate increases. At `C_n_beta = 0`, direct weathercock yaw is absent at release, but later
yaw still appears because roll rate feeds `C_n_p p_hat`. That is coupling, not a failed isolation.

## Deliberately broken rate normalization

`C_l_p` differentiates roll-moment coefficient with respect to `p_hat`, not dimensional `p`.
Omitting `b/(2V0)` gives the unit-inconsistent numeric expression
`qbar S b C_l_p = -116984.9550` in place of the correct
`L_p = -10626.1334 N*m/(rad/s)`. The correct matrix entry is `-4.25045 1/s`; the broken
expression yields the SI numeric value `-46.79398` but carries incompatible `1/s^2` units. Their
numeric-value quotient is `2V0/b = 11.00917 1/s`, not a dimensionless physical multiplier.

The resulting plot is smooth, not explosive. Peak `|p|` collapses from `3.90155` to
`0.647583 deg/s`, and peak `|phi|` from `1.64347` to `0.184170 deg`. Initial roll acceleration is
unchanged because the bad rate term is still zero at `p(0)=0`; the symptom appears only after the
state moves. This is why unit tracing must accompany visual plausibility.

## Fixed-step propagation and scope

The model evaluates `x_dot = A x` with the visible fourth-order recurrence

```text
k1 = A x_k
k2 = A (x_k + dt k1/2)
k3 = A (x_k + dt k2/2)
k4 = A (x_k + dt k3)
x_(k+1) = x_k + dt (k1 + 2 k2 + 2 k3 + k4)/6
```

on `0:0.02:25 s`, exactly 1,251 samples. This bounded linear propagation is here to expose cause
and effect. P09 must still add complete nonlinear six-degree-of-freedom force, moment, attitude,
velocity, and position integration.

## Common misconceptions

- `C_l_p` is roll damping; `C_n_beta` is weathercock restoring stability. Do not call both damping.
- A more-negative `C_l_p` does not change initial acceleration when initial roll rate is zero.
- `C_n_beta = 0` removes one direct yaw contribution, not all yaw motion in a coupled model.
- The `-r` and `g phi/V0` entries are kinematic and gravity terms, not fitted aerodynamic slopes.
- Per-radian derivatives require radians; using degree numbers multiplies the initial derivative contribution by `180/pi`.
- A smooth response can be dimensionally wrong when a nondimensional rate scale is omitted.
- Favorable derivative signs do not by themselves prove that every assembled coupled mode decays.
- One derivative changes one matrix entry directly, but coupled outputs need not remain isolated.
- These notional constant derivatives cannot establish real-aircraft modes, handling qualities, or fidelity.

## Evidence boundary

The source, deterministic contract, fixed resources, and an independent Python equation oracle can
be checked without MATLAB. Those results are static and simulated evidence. They do not prove MATLAB
syntax execution, MATLAB numerical fidelity, Live Editor order, graphics, UI callbacks, learner
understanding, bench, HIL, field, or production behavior.
