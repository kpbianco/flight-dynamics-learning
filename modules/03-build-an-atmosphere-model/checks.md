# P03 checks: Build an Atmosphere Model

## Guiding question

What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?

Ask and answer one item at a time.

## Observation check

At fixed true airspeed and standard temperature, which atmospheric quantities fall as pressure altitude rises? Why can Mach rise while dynamic pressure falls?

## Second-lever check

After resetting to `5000 m`, make the local temperature offset positive. Predict the direction of density, speed of sound, Mach, and dynamic-pressure changes before looking at the plot. Explain why pressure stays fixed in this particular pressure-altitude model.

## Limiting-case checks

- At sea level with zero temperature offset, what must temperature and pressure equal?
- At zero true airspeed, what must Mach, dynamic pressure, and equivalent airspeed equal?
- At nonzero speed, why must `0.5 rho0 EAS^2` equal the local `0.5 rho V^2`, and why is EAS below true airspeed when density is below standard sea-level density?
- Why must pressure and temperature remain continuous where the lapse layer meets the isothermal layer at 11 km?
- At fixed pressure, why does warmer air have lower density?
- If true airspeed doubles at one air state, why does Mach double while dynamic pressure quadruples?
- Why is a positive-up altitude change opposite the sign of a change in NED Down?

## Broken-case check

The deliberately broken case freezes density at its sea-level value while evaluating flight at 11 km. Explain why that assumption overpredicts dynamic pressure and every `q S C` aerodynamic force even though the true-airspeed input is correct. “The atmosphere is wrong” is not a complete diagnosis.

## Range and transfer check

Explain why the model changes equation at 11 km and rejects altitudes above 20 km, local temperature offsets outside `-100` to `+100 K`, and true airspeeds above `1000 m/s`. Then describe what measured information would be needed to replace the local temperature offset with a weather-column model or to convert geometric altitude into geopotential pressure altitude.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P03")
```

`run_checks.m` covers determinism, scalar resource bounds, sea-level and layer-boundary limits, equation identities including nonzero equivalent airspeed, two sweep regressions, zero speed, malformed inputs, recovery, and the constant-density failure. All assertions must pass before learner completion.

## Teach-back

In two sentences: first connect pressure altitude and temperature through `p`, `T`, `rho`, and speed of sound to P02 true airspeed, Mach, and dynamic pressure; then explain how the broken density assumption creates a believable but badly scaled aerodynamic result.
