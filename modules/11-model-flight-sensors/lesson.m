%% P11 - Model Flight Sensors
% Guiding question:
% What inputs, observable effects, and failure modes matter when you model Flight Sensors?

%% Read - truth is not a sensor measurement
% P10 exposed delivered actuator authority before vehicle propagation. P11
% starts downstream with prescribed truth and makes the sensor boundary
% visible. A pitch gyro carries a constant rate bias into integrated angle:
%
%   q_measured = q_truth + bias
%   theta_measured = integral(q_measured dt)
%
% A body-frame accelerometer reports specific force, not NED coordinate
% acceleration:
%
%   f_b = C_n_to_b (a_n-g_n) + eta_b
%
% Body axes are x forward, y right, z down; navigation axes are NED and
% positive pitch is nose-up. Thus a supported level sensor reports -g on
% body z-down. P11 prescribes truth internally and does not directly accept
% P10 output or implement P17 fusion or P22 latency/bus behavior.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Vary gyro bias while holding the complete
% accelerometer history fixed. Reset, then vary accelerometer teaching-error
% RMS while holding gyro and truth histories fixed. The broken comparison
% omits only gravity, exposing the zero-at-rest symptom of confusing
% coordinate acceleration with specific force.
experiment;

%% Open the live lever panel
% Use the exact reset between the bias and noise controls. Read the truth
% and measurement axes before diagnosing the broken specific-force panel.
interactive;

%% Complete the lesson
% Run run_module_checks('P11') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
