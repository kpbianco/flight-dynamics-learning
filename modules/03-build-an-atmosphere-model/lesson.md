# P03 lesson: Build an Atmosphere Model

## Guiding question

What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?

## Compounds on P02

P02 turned wind-axis true airspeed into body and North-East-Down velocity components without changing the vector's magnitude. P03 uses that air-relative magnitude—not a single component and not ground speed—to ask what the surrounding air does to Mach number and aerodynamic load scale. P03 pressure altitude is positive upward, so do not reuse the positive-Down sign convention of a NED position coordinate.

## Mental model

Think of the atmosphere as a vertical stack of air. Hydrostatic balance makes pressure decrease upward because less air remains overhead. In the gradient troposphere, standard temperature also decreases at `-0.0065 K/m`. From 11 to 20 km this lesson holds standard temperature at `216.65 K` and lets pressure continue to decay exponentially.

The visible chain is:

```text
pressure altitude h -> standard T and p
local temperature offset -> actual T
p and T -> rho = p/(R T)
T -> a = sqrt(gamma R T)
P02 true airspeed V -> Mach = V/a and q = 0.5 rho V^2
rho and V -> EAS = V sqrt(rho/rho0), preserving q at sea-level density rho0
```

The constants are `T0 = 288.15 K`, `p0 = 101325 Pa`, `g0 = 9.80665 m/s^2`, `R = 287.05287 J/(kg K)`, and `gamma = 1.4`. These are visible in `model.m`; no toolbox atmosphere function hides the calculation.

## Baseline, then one lever

The deterministic baseline uses pressure altitude `5000 m`, standard temperature offset `0 K`, and true airspeed `150 m/s`. It gives approximately `T = 255.65 K`, `p = 54.02 kPa`, `rho = 0.7361 kg/m^3`, speed of sound `320.53 m/s`, `Mach = 0.468`, dynamic pressure `q = 8.28 kPa`, and equivalent airspeed `EAS = 116.28 m/s`. EAS is the speed at standard sea-level density that produces that same dynamic pressure; it is not another velocity component.

First sweep pressure altitude while temperature offset and true airspeed stay fixed. Pressure and density decrease throughout the range. Temperature decreases to 11 km and then stays fixed, making the layer change visible. At fixed true airspeed, dynamic pressure falls with density; Mach rises while the air cools, then stays fixed in the isothermal layer.

Reset to the baseline, then sweep local temperature offset. Pressure does not change because the pressure-altitude surface is held fixed. Warmer air has lower density and dynamic pressure but higher speed of sound, so the same true airspeed has a lower Mach number.

## Scope of the temperature-offset lever

The temperature offset is a local departure applied after standard pressure is found. It is useful for seeing the ideal-gas and speed-of-sound mechanisms at one pressure altitude. It is not a complete nonstandard weather column, geometric-altitude conversion, humidity model, or altimeter model. Those require additional measured profiles and assumptions.

The interactive altitude curves remain the standard `0 K`-offset column; the selected marker alone shows the local offset. The model accepts offsets from `-100` to `+100 K` and true airspeed from `0` to `1000 m/s`, keeping every accepted scalar result finite while the UI exposes a narrower practical range.

## Deliberately broken density

The broken case carries sea-level density unchanged to 11 km. At `150 m/s`, both calculations use the same correct true airspeed, but the frozen-density result overpredicts dynamic pressure by more than a factor of three. Any aerodynamic force computed as `q S C` inherits that scale error.

This is why a plausible airspeed alone cannot validate aerodynamic loading. Pressure altitude, temperature convention, units, atmosphere range, and density calculation all matter.

## Common misconceptions

- Temperature equations use kelvin, not degrees Celsius; absolute temperature must stay above zero.
- Pressure altitude is a standard-pressure coordinate, not automatically geometric height above terrain.
- NED Down and atmosphere altitude have opposite positive directions in these lessons.
- True airspeed does not by itself set dynamic pressure; density is equally necessary.
- Temperature offset changes density and sound speed here, but it does not move the chosen pressure-altitude surface.
- The tropospheric lapse equation must not be extrapolated above 11 km; this model changes equations there and stops at 20 km.
- This dry, perfect-gas learning model does not include humidity, winds, local weather profiles, Earth-shape corrections, or certification tolerances.

## Completion standard

The learner can trace altitude and temperature through pressure, density, and speed of sound; connect P02 true airspeed to Mach and dynamic pressure; predict both lever effects; diagnose the constant-density failure; pass `run_checks.m`; and give the teach-back in `checks.md` without relying on MATLAB syntax.
