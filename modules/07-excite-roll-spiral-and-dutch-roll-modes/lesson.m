%% P07 - Excite Roll, Spiral, and Dutch-Roll Modes
% Guiding question:
% What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?

%% Read the excitation-observable-time-scale map
% P06 separated longitudinal modes by what moved and how quickly. P07 does
% the same laterally: an aileron pulse exposes fast roll-rate subsidence, a
% bank release exposes the slow spiral, and a rudder pulse exposes the
% oscillatory sideslip/yaw pair. Positive p and phi are right-wing-down;
% positive r and heading are nose-right.
disp('What inputs, observable effects, and failure modes matter when you excite Roll, Spiral, and Dutch-Roll Modes?');
disp('Inputs establish modal participation; decay rate, stability sign, and damping shape what follows.');

%% Run the deterministic experiment one transition at a time
% Observe roll, Dutch roll, and spiral separately. Sweep roll decay alone,
% read its mechanism, reset, and sweep Dutch-roll damping alone. Finish by
% diagnosing a reversed spiral-stability sign.
experiment;

%% Open the live lever panel
% Move one excitation or shaping parameter at a time. Keep roll rate, bank
% angle, heading, sideslip, and yaw rate distinct; the displayed energy is
% a normalized modal measure, not physical joules.
interactive;

%% Complete the lesson
% Run run_module_checks('P07') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
