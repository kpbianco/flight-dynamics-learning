%% P06 - Excite the Short-Period and Phugoid Modes
% Guiding question:
% What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?

%% Read the stiffness-versus-damping distinction
% P05 showed an initial restoring moment from negative C_m_alpha. P06 adds
% declared inertia and damping to expose motion in time. A brief elevator
% pulse primarily exposes fast alpha/pitch-rate motion; an airspeed/energy
% displacement exposes the slow speed/flight-path exchange. Alpha_dot is
% only approximated by q when the fast mode nearly freezes flight path.
disp('What inputs, observable effects, and failure modes matter when you excite the Short-Period and Phugoid Modes?');
disp('Restoring stiffness sets a tendency and frequency; damping decides whether the envelope fades or grows.');

%% Run the deterministic experiment one transition at a time
% Observe the fast and slow baselines, sweep short-period damping alone,
% read its mechanism, reset, and sweep phugoid damping alone. Finish by
% diagnosing a reversed damping sign.
experiment;

%% Open the live lever panel
% Move elevator-pulse amplitude, airspeed displacement, short-period
% damping, or phugoid damping one at a time. Keep alpha, pitch rate,
% flight-path angle, and altitude labels distinct.
interactive;

%% Complete the lesson
% Run run_module_checks('P06') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
