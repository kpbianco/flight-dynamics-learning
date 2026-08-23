# P04 lesson: Balance Forces in Trim

## Guiding question

What inputs, observable effects, and failure modes matter when you balance Forces in Trim?

## Compounds on P03

P03 mapped positive-up pressure altitude and temperature to air density, then combined density with
the magnitude of P02's air-relative velocity to form `q = 0.5 rho V^2`. P04 takes `rho` and true
airspeed as inputs. Ground speed or one body-axis component cannot replace that air-relative speed.
The deterministic baseline reuses P03's standard 5 km density,
`rho = 0.7361155474 kg/m^3`.

## Mental model

Use axes that follow the flight path. Forward along the path is positive, and the normal direction
points toward lift. Assume steady, wings-level motion and thrust aligned with the path. At a
flight-path angle `gamma`, positive in climb, zero acceleration requires

```text
normal:      L - W cos(gamma) = 0
along path:  T - D - W sin(gamma) = 0
```

Weight is `W = m g`. Dynamic pressure turns the air state into an aerodynamic force scale, and a
visible teaching polar turns the required lift into drag:

```text
q        = 0.5 rho V^2
CL       = W cos(gamma)/(q S)
alpha    = (CL - CL0)/CLalpha
CD       = CD0 + k CL^2
D        = q S CD
T        = D + W sin(gamma)
```

The constants are visible in `model.m`: `S = 16.2 m^2`, `CL0 = 0.25`,
`CLalpha = 5.0 per rad`, `CD0 = 0.025`, `k = 0.045`, `CLmax = 1.4`, and an
idealized constant maximum thrust of `4000 N`. No solver or toolbox call hides these equations.

## Baseline, then one lever

The baseline uses P03 density `0.7361155474 kg/m^3`, true airspeed `60 m/s`, mass `1200 kg`, and
level flight. It gives approximately `q = 1325.01 Pa`, `W = 11767.98 N`, `CL = 0.5482`,
`alpha = 3.42 deg`, parasite drag `536.63 N`, induced drag `290.32 N`, and required thrust
`826.95 N`. The normal and along-path residuals are zero by construction, the stall boundary is
`37.55 m/s`, and minimum drag occurs near `51.46 m/s` for this air state and mass.

First sweep true airspeed while density, mass, and path angle stay fixed. Lower speed reduces `q`,
so the same required lift needs larger `CL` and angle of attack. Parasite drag scales with `V^2`.
At fixed required lift, induced drag scales with `1/V^2`, so their sum has a minimum rather than
falling forever with speed.

Reset to `60 m/s`, then sweep mass. Dynamic pressure and parasite drag do not change. Weight, required
`CL`, angle of attack, and induced drag all rise; required thrust follows. This isolates mass from
the airspeed mechanism instead of changing both at once.

## Requirements are not automatically achievable

The closed-form equations still produce a requirement below stall or beyond available thrust. The
model does not silently clamp it. `liftFeasible` checks `CL <= CLmax`; `thrustFeasible` checks that
required thrust is between zero and the idealized cap. Only both together make `trimFeasible` true.
The required-thrust fraction is a ratio to that teaching cap, not a throttle command or propulsion
deck. In a steep descent, a negative required thrust means this clean configuration needs more drag
or a shallower path to hold speed—not that an ordinary engine can command negative thrust.

## Deliberately broken dynamic pressure

The broken case computes the required coefficient with `rho V^2` instead of
`q = 0.5 rho V^2`. The resulting coefficient is exactly half the correct value. When it is turned
back into lift with the correct force scale, lift is only half the required normal force and the
residual is `-0.5 W` in level flight. A tidy coefficient and a plausible angle are not evidence of
trim; both force residuals must close with consistent units and equations.

## Scope and common misconceptions

- Angle of attack is the angle between the wing reference and the air-relative velocity; it is not pitch attitude.
- Lift equals weight only for the declared steady, level point-mass case. A climb uses `W cos(gamma)` normal to the path.
- True airspeed, not ground speed, belongs in dynamic pressure.
- A zero residual is necessary but not sufficient: `CLmax` and thrust capacity can make the requirement infeasible.
- The linear lift curve and parabolic drag polar are transparent learning approximations, not wind-tunnel or certification data.
- The constant thrust cap is a feasibility marker, not a throttle-to-thrust model.
- Force trim does not solve pitching-moment or elevator trim and does not establish static stability; later modules add those questions.

## Completion standard

The learner can connect P03 density and true airspeed to dynamic pressure; close the two path-axis
force balances; predict the isolated airspeed and mass effects; distinguish an algebraic requirement
from feasible trim; diagnose the missing one-half from its residual; pass `run_checks.m`; and give
the teach-back in `checks.md` without relying on MATLAB syntax.
