%% P08 - Relate Stability Derivatives to Motion
% Guiding question:
% What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?

%% Read the derivative-to-motion chain
% P07 prescribed separate lateral modal signatures. P08 derives coupled
% beta, p, r, and phi motion from coefficient slopes. Rates are normalized
% as p-hat=p*b/(2*V0) and r-hat=r*b/(2*V0) before rate derivatives act;
% coefficients then become dimensional force and moments before they become
% state accelerations.
disp('What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?');
disp('Trace state -> normalized rate -> coefficient -> load -> acceleration -> coupled motion.');

%% Run the deterministic experiment one transition at a time
% Predict the initial signs once, inspect baseline states and derivative
% ledgers, move C_l_p alone, read its mechanism, reset, move C_n_beta alone,
% and finish by diagnosing the missing b/(2*V0) rate normalization.
experiment;

%% Open the live lever panel
% Change the initial sideslip, roll damping, or weathercock stability one at
% a time. A derivative changes one matrix entry directly, but coupling means
% several state histories can change.
interactive;

%% Complete the lesson
% Run run_module_checks('P08') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
