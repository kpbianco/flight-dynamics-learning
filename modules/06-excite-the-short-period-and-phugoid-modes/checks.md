# P06 checks: Excite the Short-Period and Phugoid Modes

## Guiding question

What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?

Ask and answer one item at a time.

## Observation check

Why does the short-period response need a seconds-scale view while the phugoid needs a
tens-of-seconds view? Use the baseline periods and name the dominant observable pair for each mode.

## Excitation check

- Set the elevator pulse to zero. Which fast observables vanish, and why does the slow response stay unchanged?
- Reset, then set the airspeed kick to zero. Which slow observables vanish, and why does the fast response stay unchanged?
- Why is the airspeed displacement an initial energy condition rather than an elevator command?

## First-lever check

At fixed elevator pulse, airspeed kick, and phugoid damping, increase `zeta_sp`. Predict the
short-period envelope ratio, damped period, and phugoid response. Explain why P05's restoring
derivative and the declared inertia hold the natural frequency fixed.

## Second-lever check

Reset `zeta_sp = 0.35`, then increase `zeta_ph` alone. Predict the phugoid envelope ratio, damped
period, and short-period response. Explain the zero-damping limiting case before describing positive
damping.

## Limiting-case and interpretation checks

- Why does `omega_d^2 + (zeta omega_n)^2 = omega_n^2` hold for each mode?
- Why does `zeta = 0` preserve the envelope without making the response identically zero?
- Why do opposite elevator pulses or opposite airspeed kicks reverse their owned responses?
- Where does the factor of two in `gamma_dot = 2g u/V0^2` come from?
- Why must `u_dot + 2 zeta_ph omega_ph u + g gamma = 0` and `h_dot = V0 gamma` hold together?
- Why are angle of attack, pitch attitude, pitch rate, and flight-path angle not interchangeable?
- Why does `C_m_alpha < 0` fail to prove that either modal envelope decays?
- Why is a decoupled analytic teaching model not evidence for a real aircraft's modal fidelity?

## Broken-case check

The deliberately broken case changes `exp(-zeta omega_n t)` to
`exp(+zeta omega_n t)` without changing the restoring derivative or natural frequency. Explain why
the peak angle-of-attack response grows beyond `97 deg` in `2.5 s`. “The sign is wrong” is
incomplete: name the damping term, the still-restoring stiffness, the growing observable, and why a
trace beyond the local `+/-5 deg` domain is a failure indicator rather than a physical prediction.

## Range and transfer check

Explain why the model bounds excitation and damping, uses fixed 401- and 481-sample grids, and keeps
negative damping out of the normal API. Then identify what P08 must add before stability derivatives
can be claimed to predict coupled aircraft motion and what P09 must add before six-degree-of-freedom
integration is present.

## Executable check

Run from the repository root:

```matlab
run_module_checks("P06")
```

`run_checks.m` covers determinism, finite fixed vector bounds, independent P05-to-P06 frequency
buildup, closed-form responses, pole and dynamic identities, signed altitude range and
`h_dot = V0 gamma` kinematics, time-scale separation, zero-input isolation, sign symmetry,
zero-damping limits, the constant-`C_L` phugoid factor of two, the inherited alpha-range bound, both
independent damping sweeps, malformed inputs, recovery, accepted corners, and the broken damping
sign. All assertions must pass before learner completion.

## Teach-back

In two sentences: first pair the elevator pulse and airspeed/energy displacement with the fast and
slow observables they excite; then explain how restoring stiffness can remain correct while a
reversed damping sign makes the short-period envelope diverge.
