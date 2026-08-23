%% P18 - Follow Waypoints
% Guiding question:
% What inputs, observable effects, and failure modes matter when you follow Waypoints?

%% Read - separate navigation, waypoint management, and course response
% P17 produced the idea of a North position estimate. P18 treats an exact
% two-dimensional North/East estimate as an input, selects one stationary
% waypoint from an ordered route, and turns its displacement into a course
% command measured clockwise from North. This is a conceptual connection;
% no P17 runtime array is consumed.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Change only the inclusive waypoint arrival
% radius, reset, then change only course-response gain. Finally compare the
% correct N/E bearing with the deliberately swapped atan2 arguments.
experiment;

%% Open the live lever panel
% Move one control at a time and use the exact reset between comparisons.
% Inspect route geometry before course response, range, or summary metrics.
interactive;

%% Complete the lesson
% Run run_module_checks('P18') from the repository root, answer one
% interpretation question at a time, then give the two-sentence teach-back
% in checks.md.
