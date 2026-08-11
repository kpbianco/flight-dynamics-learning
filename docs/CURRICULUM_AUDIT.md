# Curriculum readiness audit

**Track:** Flight Dynamics and Aerospace GNC

## Baseline conclusion

The repository has 24 uniquely identified modules in a six-phase, prerequisite-ordered sequence. P01 is the complete reference slice; P02-P24 are explicit non-runnable batch scaffolds. The learner flow is read → visualize → move one lever → visualize the delta → read/explain, followed by a broken case, checks, and teach-back.

Static structure and CLI behavior are verified in CI. MATLAB was not available during the 2026-08-11 baseline audit, so numerical execution, UI behavior, and instructional efficacy remain named validation gaps rather than implied evidence.

## Coverage and compounding order

### Phase 1: Point-mass flight

- **P01 — Turn Bank Angle into a Flight Path:** How do bank angle and airspeed determine turn rate, radius, and load factor?
- **P02 — Transform Between Aerospace Frames:** What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?
- **P03 — Build an Atmosphere Model:** What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?
- **P04 — Balance Forces in Trim:** What inputs, observable effects, and failure modes matter when you balance Forces in Trim?

### Phase 2: Stability and modes

- **P05 — See Longitudinal Static Stability:** What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?
- **P06 — Excite the Short-Period and Phugoid Modes:** What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?
- **P07 — Excite Roll, Spiral, and Dutch-Roll Modes:** What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?
- **P08 — Relate Stability Derivatives to Motion:** What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?

### Phase 3: Six-degree-of-freedom simulation

- **P09 — Integrate 6-DOF Equations:** What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?
- **P10 — Model Actuator Dynamics and Limits:** What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?
- **P11 — Model Flight Sensors:** What inputs, observable effects, and failure modes matter when you model Flight Sensors?
- **P12 — Validate Energy and Frame Conventions:** What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?

### Phase 4: Autopilots

- **P13 — Hold Pitch and Altitude:** What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?
- **P14 — Hold Roll and Heading:** What inputs, observable effects, and failure modes matter when you hold Roll and Heading?
- **P15 — Control Speed with Throttle:** What inputs, observable effects, and failure modes matter when you control Speed with Throttle?
- **P16 — Schedule Gains Across Flight Conditions:** What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?

### Phase 5: Navigation and guidance

- **P17 — Fuse INS and GPS:** What inputs, observable effects, and failure modes matter when you fuse INS and GPS?
- **P18 — Follow Waypoints:** What inputs, observable effects, and failure modes matter when you follow Waypoints?
- **P19 — Implement Pursuit Guidance:** What inputs, observable effects, and failure modes matter when you implement Pursuit Guidance?
- **P20 — Run a Dispersion Monte Carlo:** What inputs, observable effects, and failure modes matter when you run a Dispersion Monte Carlo?

### Phase 6: Flight HWIL

- **P21 — Define Flight-Computer I/O:** What inputs, observable effects, and failure modes matter when you define Flight-Computer I/O?
- **P22 — Inject Sensor and Bus Latency:** What inputs, observable effects, and failure modes matter when you inject Sensor and Bus Latency?
- **P23 — Test Fault Responses:** What inputs, observable effects, and failure modes matter when you test Fault Responses?
- **P24 — Verify an End-to-End Mission:** What inputs, observable effects, and failure modes matter when you verify an End-to-End Mission?

## Batch readiness gates

A scaffold may become `implemented` only when it has a deterministic model, a sectioned experiment, two independent parameter sweeps, one deliberately broken case, interactive controls, interpretation-focused tutor text, numerical checks, focused static tests, and evidence that says exactly what did and did not run.
