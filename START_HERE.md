# Start here

This is the Flight Dynamics and Aerospace GNC interactive MATLAB track. Run `./bin/learn status`, then
`./bin/learn start`. P01 is the reference slice; P02 through P12 are the implemented aerospace-frame,
atmosphere-model, point-mass force-trim, longitudinal-static-stability, longitudinal-mode, and
lateral-directional-mode lessons, followed by complete nonlinear rigid-body 6-DOF integration and
transparent actuator lag, rate-limit, and position-envelope modeling. P11 adds deterministic
pitch-gyro bias and body-frame accelerometer specific-force/error experiments, including an isolated
gravity-omission failure. P12 then validates the shared body/NED and specific-force meanings with an
analytic work-energy audit, independent force and heading levers, a free-fall limit, and a deliberately
wrong Down-as-height potential-energy ledger.
`curriculum/modules.json` records the contiguous implementation frontier as later modules advance
through one-to-one Portfolio Control batches. A learner session follows read → visualize → move one
lever → visualize the change → read/explain, then a broken case, checks, and teach-back.

Use `docs/CURRICULUM_AUDIT.md` for the complete phase and batch map.
