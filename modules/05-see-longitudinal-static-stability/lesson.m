%% P05 - See Longitudinal Static Stability
% Guiding question:
% What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?

%% Read the first-moment tendency
% P04 balanced forces but deliberately stopped before pitching moment.
% P05 treats that q and alpha as a locally retrimmed reference. With h
% measured aft from the MAC leading edge and nose-up C_m positive, static
% stability requires dC_m/dalpha<0: positive delta alpha must initially
% create a nose-down restoring moment.
disp('What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?');
disp('Compare CG with the neutral point, then verify the sign of the first pitching-moment response.');

%% Run the deterministic experiment one transition at a time
% Observe the baseline, sweep CG alone, read its mechanism, reset, and then
% sweep horizontal-tail area alone. Finish by diagnosing a reversed static-margin sign.
experiment;

%% Open the live lever panel
% Move CG, horizontal-tail area, angle-of-attack perturbation, or elevator
% perturbation one at a time. Elevator changes the moment input but not the
% fixed-control static-stability slope in this model.
interactive;

%% Complete the lesson
% Run run_module_checks('P05') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
