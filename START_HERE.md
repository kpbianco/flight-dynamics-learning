# Start here

This is the Flight Dynamics and Aerospace GNC interactive MATLAB track. Run `./bin/learn status`, then
`./bin/learn start`. P01 is the reference slice; P02 through P16 are the implemented aerospace-frame,
atmosphere-model, point-mass force-trim, longitudinal-static-stability, longitudinal-mode, and
lateral-directional-mode lessons, followed by complete nonlinear rigid-body 6-DOF integration and
transparent actuator lag, rate-limit, and position-envelope modeling. P11 adds deterministic
pitch-gyro bias and body-frame accelerometer specific-force/error experiments, including an isolated
gravity-omission failure. P12 then validates the shared body/NED and specific-force meanings with an
analytic work-energy audit, independent force and heading levers, a free-fall limit, and a deliberately
wrong Down-as-height potential-energy ledger.
P13 then begins the autopilot phase with a reduced-order cascaded altitude/pitch hold: outer
altitude-to-pitch gain, scheduled inner pitch response, a distinct flight-path lag, two independent
gain sweeps, an open-loop limit, and a deliberately reversed altitude/Down feedback sign. Fixed
airspeed and pitch-control effect are declared teaching boundaries rather than aircraft fidelity.
P14 carries the cascade laterally: wrapped shortest heading error commands bounded bank, a
transparent roll loop moves bank, and fixed-speed coordinated-turn kinematics move continuous
heading. Its independent gain/frequency sweeps, zero-gain limit, and raw-angle-subtraction failure
make bank authority, acceleration demand, and the `+/-180 deg` branch cut visible without claiming
wind, yaw-coupling, actuator, sensor, or full-aircraft fidelity.
P15 releases P14's fixed-speed boundary in a separate straight-and-level teaching model. A
speed-error gain commands drag feedforward plus bounded corrective thrust, a normalized first-order
throttle lag separates request from delivery, and thrust minus drag advances true airspeed. Its
independent gain and lag sweeps, exact zero-gain trim limit, and reversed-feedback idle-thrust
failure expose authority, response-rate, and positive-feedback behavior without claiming an engine
deck, identified aircraft, gain schedule, or flight-control validation.
P16 then replaces the fixed-condition gain assumption with a transparent dynamic-pressure lookup.
True airspeed and density select a manually interpolated five-knot roll schedule, while independent
condition sweeps expose nearly preserved response and changing equivalent-aileron demand. An
equal-dynamic-pressure comparison isolates a broken true-airspeed-only lookup that omits density and
selects gains that are too small for the unchanged plant. The result is a bounded frozen-condition
teaching model, not an aircraft gain schedule, robustness proof, certification artifact, or P14/P15
runtime adapter.
`curriculum/modules.json` records the contiguous implementation frontier as later modules advance
through one-to-one Portfolio Control batches. A learner session follows read → visualize → move one
lever → visualize the change → read/explain, then a broken case, checks, and teach-back.

Use `docs/CURRICULUM_AUDIT.md` for the complete phase and batch map.
