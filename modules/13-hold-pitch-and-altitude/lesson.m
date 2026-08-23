%% P13 - Hold Pitch and Altitude
% Guiding question:
% What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?

%% Read - cascade altitude outside pitch
% P12 established geometric altitude h=-NED Down. P13 turns that declared
% sign into feedback and keeps each transition visible:
%
%   altitude error -> pitch command -> pitch-control effect -> pitch
%                  -> flight-path angle -> climb rate -> altitude
%
% The outer gain K_h maps metres to radians. The inner pitch-loop natural
% frequency sets how quickly pitch follows its command. Theta and gamma are
% perturbations about level trim, and pitch theta is not
% flight-path angle gamma; a declared first-order path response separates
% them before h_dot=V*sin(gamma) changes altitude.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Sweep only altitude-to-pitch gain, reset,
% then sweep only inner-loop natural frequency. Finally reverse only the
% altitude-feedback sign to reproduce the P12 Down/altitude convention
% failure as positive feedback.
experiment;

%% Open the live lever panel
% Use reset between the two sliders. The failure switch changes only the
% feedback sign; switch it back before comparing the two gain levers.
interactive;

%% Complete the lesson
% Run run_module_checks('P13') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
