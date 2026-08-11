# Lesson: Turn Bank Angle into a Flight Path

## Guiding question

How do bank angle and airspeed determine turn rate, radius, and load factor?

## Mental model

In a coordinated level turn, bank tilts the lift vector. Its horizontal component bends the trajectory while the vertical component must still support weight.

## What to manipulate

Use `interactive.m`. Change one lever at a time before combining effects.

## First observation

Increase bank angle and watch turn radius shrink, turn rate rise, and load factor increase. Increase airspeed and see radius grow with speed squared.

## Common mistakes

- Bank angle does not directly command a fixed turn radius independent of speed.
- A level coordinated turn requires more lift than straight flight.
- Point-mass kinematics do not capture stall, actuator, or aerodynamic limits.

## Completion standard

The learner can explain the baseline, identify what each lever changes, diagnose the deliberately broken case, and pass `run_checks.m`.
