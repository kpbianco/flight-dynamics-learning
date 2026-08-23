%% P15 - Control Speed with Throttle
% Guiding question:
% What inputs, observable effects, and failure modes matter when you control Speed with Throttle?

%% Read - close speed feedback through throttle and forward force
% P14 treated true airspeed as a fixed 60 m/s condition. P15 turns that
% condition into a controlled state:
%
%   speed error -> bounded thrust request -> delivered throttle
%               -> thrust minus drag -> acceleration -> true airspeed
%
% The level-flight drag balance comes conceptually from P04, and the
% requested-versus-delivered distinction comes conceptually from P10. P15
% does not run those modules or accept their histories.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Sweep only speed-feedback gain, reset,
% then sweep only throttle time constant. Finally reverse only the feedback
% sign and observe idle throttle, falling speed, and growing proper error.
experiment;

%% Open the live lever panel
% Use reset between the two sliders. The failure switch changes only speed
% feedback sign; restore correct feedback before comparing the two levers.
interactive;

%% Complete the lesson
% Run run_module_checks('P15') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
