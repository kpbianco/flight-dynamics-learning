# P12 checks: Validate Energy and Frame Conventions

## Guiding question

What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?

Answer from the observed trajectory and ledgers, not from MATLAB syntax. A complete answer names the frame map, identifies non-gravity work as the cause of mechanical-energy change, and diagnoses the wrong sign that appears when positive NED Down is mistaken for positive height.

## What to observe

Start with `model(1.5,30)`. The fixed 30 deg nose-up attitude maps body-forward velocity and non-gravity specific force toward positive North, positive East, and negative Down. Check all three descriptions of the same motion:

- `C_body_to_ned` maps body components into North-East-Down components, and its transpose maps them back.
- `0.5*m*(v dot v)` and `m*f dot v` agree whether the dot product is evaluated in body or NED coordinates.
- altitude is `h=-Down`, so `U=m*g*h=-m*g*Down` and `E(t)-E(0)` equals accumulated non-gravity work.

P11 supplied the ideal accelerometer quantity `f_b=C_n_to_b*(a_n-g_n)`. P12 rearranges that same relationship as `a_n=C_body_to_ned*f_b+g_n`; this is a conceptual prerequisite, not a compatibility adapter or a new P11 API.

## Controlled levers

1. Reset heading to 30 deg, then sweep body-x specific force through `[0 0.75 1.5 2.25 3]` m/s^2. More force produces more positive non-gravity work, more horizontal range, and a larger apex gain because a nose-up body-x force has an upward, negative-Down component. The zero-force limit is free fall: the ideal accelerometer quantity, non-gravity power, and work are zero while mechanical energy remains constant.
2. Reset force to 1.5 m/s^2, then sweep heading through `[-90 -30 0 30 90]` deg. This actively yaws the body and trajectory within fixed NED rather than passively relabeling one trajectory. Uniform gravity and the absence of a horizontal asymmetry make the vertical and scalar histories yaw-symmetric; a proper DCM preserves body/NED norms and dot products. Signed North/East position, velocity, and force histories rotate, while altitude, body-frame velocity history, speed, power, work, energy, and the broken-case residual remain fixed.

Use `interactive.m` to move one lever at a time. The reset button restores 1.50 m/s^2 and 30 deg before the second experiment.

## Deliberately broken case

The broken ledger replaces the correct height `h=-Down` with `h=+Down` while preserving the trajectory, DCM, force, velocity, power, and work. It therefore begins with zero residual at the shared datum but develops the exact false residual

```text
2*m*g*(Down(t)-Down(0)).
```

During the baseline climb this looks like more than one megajoule of unexplained energy loss even though the correct ledger closes. That symptom isolates a potential-energy sign error rather than a force, integration, or frame-rotation error.

## Executable validation

Run from this module directory:

```matlab
run_checks
```

`run_checks.m` independently reconstructs the DCM, every analytic position and velocity sample, body/NED kinetic energy and power, every trapezoidal work interval, the work-energy balance, and the broken residual. It also pins a nonzero signed 30 deg pitch/heading case, checks free fall and due-North/East/West limits, requires every intermediate heading-sweep path, velocity, and force to rotate by the commanded angle, isolates both five-point sweeps, tests all four accepted input corners and a capped nine-case representative grid, and rejects out-of-range, nonscalar, complex, NaN, and Inf inputs before proving deterministic recovery.

The model is synchronous and fixed at 301 samples over 300 intervals; it has no asynchronous worker, blocking wait, external I/O, persistent state, cancellation path, or runtime timeout to exercise. Cancellation and timeout behavior are therefore not applicable. Rejected calls cannot partially mutate state, so recovery is a fresh valid call. The model writes no data and performs no migration, making backup/restore and data rollback not applicable. Compatibility is limited to preserving the P11 frame/specific-force meanings and repository learner commands.

Repository-level checks are:

```bash
python3 -m unittest -v tests.test_p12_module tests.test_curriculum tests.test_learn_cli
MATLAB_LEARNING_VERIFY_PROFILE=contract python3 scripts/verify.py
MATLAB_LEARNING_VERIFY_PROFILE=quick python3 -m unittest discover -s tests -v
MATLAB_LEARNING_VERIFY_PROFILE=full ./scripts/agent-verify.sh
```

These are static and simulated checks unless a named MATLAB runtime is actually used. They do not establish physical hardware, HIL, field, real-time, deployment, or production evidence.

## Teach-back

In two sentences, state which way `C_body_to_ned` maps components and why speed, kinetic energy, and power do not change under that proper rotation. Then explain why non-gravity work changes mechanical energy and why using positive Down as height creates a false residual even when all kinematics remain identical.

## Rollback and later batches

Rollback removes only the P12 module implementation and its P12-specific tests/evidence, restores P12 to `scaffolded` with evidence `none`, and reverts the P12 navigation entries. If later batches depend on P12, coordinate their lifecycle state before rollback; persisted P12 checks intentionally assert permanent P12 facts and the P11 prerequisite rather than assuming P12 remains the latest frontier.
