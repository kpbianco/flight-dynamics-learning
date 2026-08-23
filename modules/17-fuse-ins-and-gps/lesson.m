%% P17 - Fuse INS and GPS
% Guiding question:
% What inputs, observable effects, and failure modes matter when you fuse INS and GPS?

%% Read - separate high-rate prediction from absolute correction
% P16 closed the autopilot phase with a state-feedback teaching model. P17
% begins navigation by asking where an estimated position and velocity can
% come from. A one-dimensional INS integrates North acceleration at 50 Hz;
% one-Hz GPS fixes correct its drift through a visible innovation.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Sweep only INS acceleration bias, reset,
% then sweep only deterministic GPS position-error RMS. Finally compare the
% correct innovation gate with an accept-all broken case on the same outlier.
experiment;

%% Open the live lever panel
% Move one sensor-error control, inspect prediction and correction, then use
% the exact reset. Select INS-only once and the broken ungated mode once
% before restoring gated fusion.
interactive;

%% Complete the lesson
% Run run_module_checks('P17') from the repository root, then answer the
% interpretation questions and give the two-sentence teach-back in checks.md.
