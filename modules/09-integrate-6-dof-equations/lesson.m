%% P09 - Integrate 6-DOF Equations
% Guiding question:
% What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?

%% Read the complete rigid-body state chain
% P08 connected stability derivatives to force, moment, and four lateral
% states. P09 advances NED position, body velocity, scalar-first
% body-to-NED quaternion, and body rates. Applied non-gravity force changes
% translation; moment changes angular momentum; quaternion attitude maps
% rotating body components into the fixed NED view.
disp('What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?');
disp(['Trace force and moment -> body acceleration and rate -> quaternion ' ...
    'attitude -> NED velocity and position.']);

%% Run the deterministic experiment one transition at a time
% Make one prediction, inspect the baseline trajectory and states, move
% forward force alone, read its mechanism, reset, move moment alone, and
% diagnose the omitted -omega cross velocity transport term.
experiment;

%% Open the live lever panel
% Reset between the force-pulse and moment-pulse controls. Watch the direct
% load first, then body states, quaternion-derived attitude, NED path, and
% the complete-equation residual.
interactive;

%% Complete the lesson
% Run run_module_checks('P09') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
