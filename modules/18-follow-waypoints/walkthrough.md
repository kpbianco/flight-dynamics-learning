# P18 walkthrough: Follow Waypoints

## Guiding question

What inputs, observable effects, and failure modes matter when you follow Waypoints?

Follow one plot or processing transition at a time. Run the fixed experiment before opening the
interactive controls.

1. Read the P17 prerequisite connection in `lesson.md`. Explain why P18 needs a navigation position
   but does not consume P17 estimator histories.
2. State the conventions: position is `[North,East]` in meters, course is clockwise from North,
   groundspeed is `25 m/s`, sample time is `0.1 s`, and the targets are stationary and ordered.
3. Trace active waypoint, `Delta North`, `Delta East`, `atan2(Delta East,Delta North)`, shortest
   course error, bounded course rate, planar motion, and inclusive arrival in that order.
4. Make the experiment's one prediction: if arrival radius grows while route and response stay
   fixed, will the recorded path get longer or shorter?
5. Run only the baseline section of `experiment.m`. Inspect the dashed route and waypoint order
   before looking at the flown path.
6. Add the flown path and arrival circles. Confirm the four target captures occur near `14.8`,
   `29.4`, `41.7`, and `59.2 s`, each at or inside `30 m`.
7. Inspect only commanded and actual course. Relate the delayed corner response to finite course
   rate rather than to waypoint motion or wind, neither of which exists here.
8. Inspect shortest course error and bounded rate. Confirm the rate never exceeds `12 deg/s` even
   when a new leg changes bearing by roughly `90 deg`.
9. Inspect only active-waypoint range and index. Confirm the index advances in order when the range
   crosses the inclusive arrival threshold; no later waypoint can be selected early.
10. Reset gain to `0.8 1/s` and correct bearing mode. Sweep arrival radius through
    `[10,20,30,50,80] m`.
11. Follow flown distance and each capture range. Explain why a larger decision circle produces
    earlier switching and more corner cutting, not greater spatial accuracy.
12. Reset radius to `30 m`. Sweep course-response gain through `[0,0.2,0.4,0.8,1.2] 1/s`.
13. Inspect the zero-gain trace alone. Confirm it captures W2 while flying North, then cannot turn
    toward W3; course rate, East position, and course remain exactly zero.
14. Add the positive-gain traces one at a time. Observe faster initial capture, then a plateau as
    more samples demand the same `12 deg/s` course-rate limit.
15. Restore `model(30,0.8,1)`. Use `25*0.1=2.5 m` to reconstruct each active displacement and
    `25/deg2rad(12)` to explain the roughly `119.366 m` saturated turn-radius scale.
16. Record the first correct bearing command: the due-North target produces `0 deg`.
17. Select the deliberately broken swapped-bearing mode. Inspect only the first ten seconds: the
    same due-North target now commands `+90 deg` East.
18. Follow the broken route geometry. Confirm it captures no target, never advances beyond active
    index 2, and misses W2 by roughly `296.869 m` despite tracking its wrong course command.
19. Use the exact reset. Move radius alone, reset; move response gain alone, reset; then restore
    correct N/E bearing after viewing the broken case.
20. Run `run_module_checks("P18")` from the repository root, answer one interpretation question at
    a time, and give the two-sentence teach-back from `checks.md`.

The retained numbers are static and independent simulated-oracle references. MATLAB must run the
lesson separately before anyone claims MATLAB runtime, plots, UI callbacks, reset behavior, or
MATLAB numerical validation.
