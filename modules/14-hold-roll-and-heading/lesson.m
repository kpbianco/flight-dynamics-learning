%% P14 - Hold Roll and Heading
% Guiding question:
% What inputs, observable effects, and failure modes matter when you hold Roll and Heading?

%% Read - cascade heading outside bank
% P13 established the outer-objective/inner-attitude pattern. P14 turns
% circular heading error into bank command, lets a transparent inner loop
% move bank, then uses coordinated-turn kinematics to move heading:
%
%   circular heading error -> bounded bank command -> bank response
%                          -> heading rate -> continuous heading
%
% Positive bank is right-wing-down and positive heading is nose-right.
% Heading is integrated continuously and wrapped only for display and error;
% +170 deg to -170 deg is the +20 deg shortest path.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Sweep only heading-to-bank gain, reset,
% then sweep only inner roll-loop natural frequency. Finally replace only
% circular error with raw displayed-angle subtraction and observe the
% saturated wrong-way turn through the end of the fixed trace.
experiment;

%% Open the live lever panel
% Use reset between the two sliders. The failure switch changes only the
% error calculation; restore wrapped shortest-path feedback before comparing
% the two levers.
interactive;

%% Complete the lesson
% Run run_module_checks('P14') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
