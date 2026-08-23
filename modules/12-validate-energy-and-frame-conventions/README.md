# P12 — Validate Energy and Frame Conventions

**Track:** Flight Dynamics and Aerospace GNC

**Phase 3:** Six-degree-of-freedom simulation

**Status:** implemented

## Guiding question

What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?

## Physical mental model

P09 propagated body velocity into North-East-Down (NED) motion, and P11 used the ideal
accelerometer equation `f_b=C_n_to_b*(a_n-g_n)`. P12 closes that conceptual loop by rearranging the
same relation and auditing its work-energy consequences:

```text
a_n       = C_body_to_ned f_b + g_n
power     = m f_b dot v_b = m (C_body_to_ned f_b) dot v_n
E         = 0.5 m (v dot v) + m g h
E(t)-E(0) = integral(power dt)
```

The body frame is `x` forward, `y` right, `z` down. The navigation frame is NED, positive pitch is
nose-up, and heading is clockwise from North toward East. A proper direction-cosine matrix (DCM)
preserves vector norms and dot products, so kinetic energy and non-gravity power cannot depend on
whether their components are written in body or NED axes.

NED Down is a coordinate, not altitude. With geometric altitude `h=-Down`, gravitational potential
energy is `U=m*g*h=-m*g*Down`. A sign error can leave a trajectory smooth and every DCM check green
while corrupting the energy ledger.

This is a self-contained validation harness, not a package adapter. P11 produces gyro and
accelerometer measurements on a different prescribed truth history; P12 does not accept those
measurements, estimate a state, or feed a controller. The connection is the shared declared
specific-force and frame meaning.

## Deterministic experiment

The audit body uses P09's `1200 kg` mass and `60 m/s` initial speed. It begins at `1000 m` altitude
with fixed `30 deg` nose-up pitch, zero roll, and selected heading. The selected constant body-x
non-gravity specific force is rotated into NED, gravity `[0;0;9.80665] m/s^2` is added, and the
constant-acceleration trajectory is evaluated analytically on `0:0.02:6 s`: exactly 301 samples and
300 retained intervals. The body attitude remains fixed so the lesson isolates frame and energy
bookkeeping from attitude dynamics.

At the baseline `f_x=1.5 m/s^2`, heading `30 deg`:

- the first DCM column is `[0.75;0.4330127019;-0.5]`;
- initial NED velocity is `[45;25.9807621;-30] m/s`;
- NED non-gravity specific force is `[1.125;0.6495191;-0.75] m/s^2`;
- total NED acceleration is `[1.125;0.6495191;9.05665] m/s^2`;
- the analytic apex occurs at `3.3124831 s`, `49.6872464 m` above the initial altitude;
- final NED position is `[290.25;167.5759156;-1016.9803] m`;
- final NED velocity is `[51.75;29.8778764;24.3399] m/s`;
- accumulated non-gravity work is `537732.27 J`, exactly matching mechanical-energy change within
  the retained arithmetic tolerance.

Those are deterministic equation references. They are not MATLAB-runtime, numerical-fidelity,
graphics, UI, aircraft, sensor, bench, HIL, or flight evidence.

## Two independent levers

1. Hold heading at `30 deg` and sweep body-x non-gravity specific force through
   `[0,0.75,1.5,2.25,3] m/s^2`. The DCM and initial state remain fixed. Work, horizontal range, and
   apex gain increase, while `E-E0-work` remains near zero. At `f_x=0`, the exact free-fall limit has
   zero ideal accelerometer output, power, and work, and constant mechanical energy.
2. Reset `f_x=1.5 m/s^2` and sweep heading through `[-90,-30,0,30,90] deg`. This actively yaws the
   fixed-attitude body and its trajectory relative to the fixed NED axes; it is not a passive
   coordinate relabeling. North and East histories rotate, while body velocity, Down/altitude,
   speed, kinetic and potential energy, power, work, apex, and both correct/broken energy histories
   remain fixed. Uniform gravity and the absence of wind or another horizontal asymmetry explain
   the unchanged vertical/scalar histories; proper-DCM dot-product preservation explains the
   body/NED scalar agreement.

The interactive reset button restores exactly `1.5 m/s^2` and `30 deg` between lever experiments.

## Deliberately broken Down-as-height ledger

The broken calculation substitutes `h=+Down` into potential energy while preserving the complete
trajectory, DCM, body/NED velocities, specific force, power, and accumulated work. Its residual is
not arbitrary:

```text
broken balance residual = 2 m g [Down(t)-Down(0)]
```

Correct and broken balances are both zero at the initial datum, which shows why one-point checks are
weak. During the baseline climb the broken ledger reports more than `1 MJ` of false unexplained
energy while the correct work-energy balance closes. The symptom is a sign-convention defect, not
drag, instability, integration drift, or sensor noise.

## Run

From MATLAB with the repository root as the current folder:

```matlab
launch_lesson("P12")
run_module_checks("P12")
```

The implementation uses base MATLAB arithmetic, explicit matrices, analytic kinematics, and a
visible trapezoidal work check. It does not use an ODE solver, Aerospace Toolbox, Simulink, random
source, file or network service, device, timer, future, or parallel worker. There is no background
task to time out or cancel.

## Files

- `lesson.m` — sectioned entry point and concise convention narrative.
- `model.m` — guarded deterministic DCM, analytic motion, work/energy ledgers, and broken comparison.
- `experiment.m` — baseline views, two isolated sweeps, free-fall limit, and Down-sign failure.
- `interactive.m` — specific-force/heading controls, exact reset, and immediate closure views.
- `lesson.md` and `walkthrough.md` — prerequisite transfer, mechanisms, misconceptions, and order.
- `checks.md` and `run_checks.m` — interpretation prompts and independent numerical invariants.

Static source inspection and an independent Python equation oracle can validate structure and
simulated reference behavior without MATLAB. They do not establish MATLAB execution, MATLAB
numerical behavior, Live Editor order, plots, callbacks, learner understanding, hardware, bench,
HIL, field, release, deployment, or production evidence.
