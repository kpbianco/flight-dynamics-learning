# P14 walkthrough: Hold Roll and Heading

## Guiding question

What inputs, observable effects, and failure modes matter when you hold Roll and Heading?

Follow one plot or transition at a time. Run the fixed experiment before opening interactive
controls.

1. Read the P13 connection in `lesson.md`. Trace the shared cascade pattern, then explain why P14
   does not accept a P13 history or run its altitude controller.
2. State the signs: positive bank is right-wing-down and positive heading is nose-right. Treat
   bank rate as an Euler-angle rate inside the declared level-turn approximation.
3. Write `wrap(x)=mod(x+pi,2*pi)-pi`, then identify its half-open `[-180,180)` range. Explain why
   exactly `180 deg` does not have a unique shortest direction.
4. Make one prediction: for a displayed command from `+170 deg` to `-170 deg`, should the shortest
   turn initially bank left or right?
5. Run the baseline section of `experiment.m`. Confirm fixed `60 m/s` speed, `K_psi=0.5 rad/rad`,
   `omega_phi=2.4 rad/s`, and the command at `1 s`.
6. Inspect only the continuous-heading figure. Follow `+170 deg` toward the nearest equivalent
   target `+190 deg`; do not mistake the wrapped display's branch cut for physical motion.
7. At the command sample, compare `-340 deg` raw subtraction with `+20 deg` circular error. Explain
   why correct bank command is `+10 deg` while bank has not yet moved.
8. Inspect only bank command and bank. At `1.5 s`, compare about `3.76565 deg` bank with the still
   larger command and explain the inner-loop lag.
9. Add only heading rate. Relate its positive sign to right bank through
   `psi_dot=g tan(phi)/V`, with radians inside `tan`.
10. Reset roll frequency to `2.4 rad/s` and error mode to wrapped. Sweep heading gain through
    `[0,0.25,0.5,0.75,1] rad/rad`.
11. Start with `K_psi=0`. Confirm the displayed command changes while bank, heading rate, and
    continuous heading remain exactly at trim; name this the open-heading-loop limit.
12. Inspect shortest error at `10 s`. Explain why more radians of bank command per radian of
    heading error reduces early error.
13. Inspect peak bank and bank-command saturation. Explain why the `12 deg` envelope makes outer
    gain an authority trade rather than free capture speed.
14. Reset `K_psi=0.5 rad/rad` and wrapped error. Sweep inner natural frequency through
    `[1.2,1.8,2.4,3.0,3.6] rad/s`.
15. Inspect bank at `1.5 s` and bank-tracking RMS before looking at heading. Explain how fixed
    damping with higher `omega_phi` makes the inner response faster.
16. Inspect peak bank acceleration. State the trade: faster bank tracking costs acceleration
    demand, without proving actuator feasibility or aircraft bandwidth.
17. Run the deliberately broken raw-subtraction case. Confirm state histories match correct mode
    through command onset while the selected errors produce `+10 deg` versus `-12 deg` bank
    commands there.
18. Follow only continuous heading change. Observe the broken case travel more than `110 deg` left
    while the nearest command is `+20 deg` right.
19. Follow only independent shortest error. Diagnose a circular-coordinate failure: proper error
    grows while the broken controller's raw arithmetic follows a contracting `-340 deg` route.
20. Open `interactive.m`. Move outer gain alone, reset; move roll frequency alone, reset; then use
    raw subtraction once and restore wrapped error.
21. Run `run_module_checks("P14")` from the repository root, then give the two-sentence teach-back
    from `checks.md`.

The numeric values are retained static and independent simulated-oracle references. MATLAB must run
the lesson separately before anyone claims MATLAB runtime, plot, UI callback, or MATLAB numerical
validation.
