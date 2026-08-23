%% P16 - Schedule Gains Across Flight Conditions
% Guiding question:
% What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?

%% Read - turn true airspeed and density into a scheduling condition
% P15 made true airspeed visible. P16 combines it with density through
% qbar=0.5*rho*V^2, then manually interpolates a five-knot roll-gain table.
% The actual plant and the lookup condition remain separate so a wrong or
% stale scheduling input has an observable symptom.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Sweep only true airspeed at reference
% density, reset, then sweep only density at reference airspeed. Finally
% compare equal-dynamic-pressure conditions and omit density only in the
% deliberately broken lookup.
experiment;

%% Open the live lever panel
% Move one condition control, inspect roll and equivalent aileron, then use
% the exact reset. Use fixed gains as a comparison, and use the broken
% true-airspeed-only source once before restoring dynamic-pressure lookup.
interactive;

%% Complete the lesson
% Run run_module_checks('P16') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
