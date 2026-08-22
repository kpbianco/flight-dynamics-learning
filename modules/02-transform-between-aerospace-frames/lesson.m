%% P02 - Transform Between Aerospace Frames
% Guiding question:
% What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?

%% Read the physical model
% P01 drew a North/East flight path from speed and heading. P02 opens the
% coordinate step hidden inside that plot: an air-relative velocity starts
% as [V;0;0] in wind axes, becomes [u;v;w] in body axes through angle of
% attack and sideslip, and becomes [N;E;D] through roll, pitch, and yaw.
% The arrow is unchanged; only its coordinates change.
disp('What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?');
disp('Track one physical velocity through wind, body, and North-East-Down coordinates.');

%% Run the deterministic experiment one section at a time
% Observe the baseline before moving yaw. Reset yaw before moving sideslip.
% Read each mechanism section only after describing the changed view.
experiment;

%% Open the live lever panel
% Move one control at a time. Speed sets vector length; alpha and beta set
% wind-to-body direction; roll, pitch, and yaw set body-to-NED attitude.
interactive;

%% Complete the lesson
% Run run_module_checks('P02') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
