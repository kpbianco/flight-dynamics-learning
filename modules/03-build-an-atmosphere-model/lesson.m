%% P03 - Build an Atmosphere Model
% Guiding question:
% What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?

%% Read the physical model
% P02 supplied the direction and magnitude of an air-relative velocity.
% P03 supplies the air around it. Pressure altitude and a local temperature
% offset determine temperature, pressure, density, and speed of sound; true
% airspeed then determines Mach, dynamic pressure, and equivalent airspeed.
% Altitude here is positive up, unlike the positive-Down NED coordinate.
disp('What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?');
disp('Build p, T, rho, and sound speed visibly before using them to interpret airspeed.');

%% Run the deterministic experiment one section at a time
% Observe the baseline, sweep altitude alone, reset it, and then sweep local
% temperature offset alone. Read each mechanism after describing the plot.
experiment;

%% Open the live lever panel
% Move pressure altitude, temperature offset, or true airspeed one at a time.
% The model is valid only from 0 to 20 km geopotential pressure altitude.
interactive;

%% Complete the lesson
% Run run_module_checks('P03') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
