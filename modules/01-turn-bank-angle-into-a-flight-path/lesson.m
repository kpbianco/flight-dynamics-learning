%% P01 - Turn Bank Angle into a Flight Path
% Guiding question:
% How do bank angle and airspeed determine turn rate, radius, and load factor?
%
% Mental model:
% In a coordinated level turn, bank tilts the lift vector. Its horizontal component bends the trajectory while the vertical component must still support weight.

%% Read the baseline lesson
disp('How do bank angle and airspeed determine turn rate, radius, and load factor?');
disp('In a coordinated level turn, bank tilts the lift vector. Its horizontal component bends the trajectory while the vertical component must still support weight.');

%% Run the deterministic experiment
experiment;

%% Open the live lever panel
% Move one control at a time and connect the visible change to the model.
interactive;
