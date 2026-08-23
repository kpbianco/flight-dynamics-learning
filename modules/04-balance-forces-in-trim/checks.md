# P04 checks: Balance Forces in Trim

## Guiding question

What inputs, observable effects, and failure modes matter when you balance Forces in Trim?

Ask and answer one item at a time.

## Observation check

At fixed P03 density and aircraft mass, why do required `CL` and angle of attack rise as true
airspeed falls? Why can parasite drag fall at the same time that induced drag rises?

## Second-lever check

After resetting true airspeed to `60 m/s`, increase mass alone. Predict the direction of weight,
dynamic pressure, required `CL`, parasite drag, induced drag, and required thrust before looking at
the plot. Explain which quantities remain fixed and why.

## Limiting-case checks

- In steady level flight, why must `L = W` and `T = D`?
- If true airspeed doubles at fixed density and required lift, why does `q` quadruple, required `CL` quarter, parasite drag quadruple, and induced drag quarter?
- If mass doubles at fixed density and speed, why does required `CL` double while induced drag quadruples?
- At the analytic stall speed, why must required `CL` equal `CLmax`, the lift-coefficient margin equal zero, and the inclusive lift-feasibility gate still pass?
- At minimum drag, why must parasite and induced drag be equal for this parabolic polar?
- For equal positive and negative path angles, why is normal-force demand the same while required thrust differs by `2 W sin(gamma)`?
- Why can the equations close to zero residual while `trimFeasible` is false?

## Broken-case check

The deliberately broken case omits the one-half in `q = 0.5 rho V^2` when computing required lift
coefficient. Explain why this commands half the required lift and produces a negative half-weight
normal residual. “The coefficient is wrong” is not a complete diagnosis: name the inconsistent force
scale and the observable symptom.

## Range and transfer check

Explain why this learning model bounds density, true airspeed, mass, and flight-path angle while still
reporting infeasible aerodynamic requirements inside those arithmetic bounds. Then explain why the
required-thrust fraction is not an engine throttle command and why negative required thrust in a
descent calls for more drag or a shallower path. Finally, name the pitching-moment, elevator, and
stability information needed to extend point-mass force trim into full-aircraft trim.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P04")
```

`run_checks.m` covers determinism, finite scalar resource bounds, independent force equations,
stall and minimum-drag limits, speed/mass/density scaling, both experiment sweeps, path-angle
symmetry, distinct feasibility failures, malformed inputs, recovery, and the broken dynamic-pressure
factor. All assertions must pass before learner completion.

## Teach-back

In two sentences: first connect P03 density and true airspeed through dynamic pressure to the lift,
drag, angle-of-attack, and thrust requirements; then explain how force residuals and feasibility
margins expose a broken or unachievable trim result even when its coefficients look plausible.
