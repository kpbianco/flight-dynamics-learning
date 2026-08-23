%% P04 - Balance Forces in Trim
% Guiding question:
% What inputs, observable effects, and failure modes matter when you balance Forces in Trim?

%% Read the path-axis force model
% P03 supplied density and true airspeed, so dynamic pressure q is known.
% P04 balances lift against the normal component of weight and thrust
% against drag plus the along-path component of weight. Required values are
% kept separate from CL and thrust limits so algebra is not mistaken for an
% achievable trim state.
disp('What inputs, observable effects, and failure modes matter when you balance Forces in Trim?');
disp('Close both force residuals, then check whether lift and thrust limits make the requirement feasible.');

%% Run the deterministic experiment one transition at a time
% Observe the baseline, sweep true airspeed alone, read its mechanism, reset,
% and then sweep mass alone. Finish by diagnosing the broken q calculation.
experiment;

%% Open the live lever panel
% Move P03 air density, true airspeed, mass, or flight-path angle one at a
% time. The thrust-capacity ratio is not an engine throttle model.
interactive;

%% Complete the lesson
% Run run_module_checks('P04') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
