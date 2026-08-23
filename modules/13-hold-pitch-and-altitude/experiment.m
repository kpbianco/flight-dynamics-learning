%% P13 - Hold Pitch and Altitude
% Guiding question:
% What inputs, observable effects, and failure modes matter when you hold Pitch and Altitude?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P13 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - an outer altitude loop commands an inner pitch loop
% P12 established geometric altitude h=-Down. P13 closes feedback around
% that sign convention:
%
%   e_h       = h_command-h
%   theta_c   = sat(K_h e_h)
%   u_theta   = sat(K_p(theta_c-theta)+K_ff theta_c-K_q q)
%   h_dot     = V sin(gamma),  gamma_dot=(theta-gamma)/tau_gamma
%
% Pitch theta and flight-path angle gamma are different states. The inner
% loop moves pitch first; the declared lift/path lag then moves gamma and
% altitude. Positive u_theta means a nose-up pitch-control effect; theta and
% gamma are perturbations about level-trim references.
disp('P13 traces altitude error through pitch command, control effect, path angle, and altitude.');
disp(['Predict once: after a positive altitude step, which signal should ' ...
    'move first: pitch angle, flight-path angle, or altitude?']);

%% Baseline - capture a 30 m altitude step with a stable cascade
baseline=model(0.004,2.4,1);
commandIndex=find(baseline.time_s>=baseline.commandStepTime_s,1,'first');
fiveSecondIndex=find(baseline.time_s>=5,1,'first');
fprintf(['Baseline: K_h %.4f rad/m, pitch omega_n %.1f rad/s, ' ...
    'fixed speed %.1f m/s, ' ...
    'command %.1f m to %.1f m at %.1f s.\n'], ...
    baseline.altitudeGain_rad_per_m, ...
    baseline.pitchNaturalFrequency_radps, ...
    baseline.trueAirspeed_mps, ...
    baseline.initialAltitude_m,baseline.altitudeCommand_m(end), ...
    baseline.commandStepTime_s);
fprintf(['At 1.5 s: pitch %.3f deg and path angle %.3f deg. ' ...
    'At 5 s: altitude error %.3f m.\n'], ...
    baseline.pitchAngle_deg(find(baseline.time_s>=1.5,1,'first')), ...
    baseline.flightPathAngle_deg(find(baseline.time_s>=1.5,1,'first')), ...
    baseline.altitudeError_m(fiveSecondIndex));
fprintf(['Final error %.4f m; overshoot %.3f m; pitch tracking RMS %.3f deg; ' ...
    'peak pitch-control command %.3f deg.\n'], ...
    baseline.finalAltitudeError_m,baseline.peakAltitudeOvershoot_m, ...
    baseline.pitchTrackingRMS_deg,baseline.peakPitchControlCommand_deg);
assert(baseline.sampleCount==1501 && baseline.intervalCount==1500 && ...
    all(baseline.altitude_m(1:commandIndex)==baseline.initialAltitude_m) && ...
    baseline.pitchCommand_deg(commandIndex)>0 && ...
    baseline.pitchAngle_deg(commandIndex)==0 && ...
    baseline.finalAltitudeError_m<0.02 && ...
    baseline.finalAltitudeError_m>-0.02 && baseline.settledByEnd, ...
    'The baseline must hold before the step and capture the commanded altitude.');

%% Baseline view 1 - altitude responds after the command
figure('Name','P13 baseline altitude capture');
plot(baseline.time_s,baseline.altitudeCommand_m,'k--','LineWidth',1.4); hold on;
plot(baseline.time_s,baseline.altitude_m,'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Geometric altitude, h=-Down (m)');
legend({'altitude command','altitude response'},'Location','best');
title('Outer loop captures a 30 m altitude step');

%% Baseline view 2 - follow one transition through the cascade
figure('Name','P13 baseline pitch path and control');
subplot(2,1,1);
plot(baseline.time_s,baseline.pitchCommand_deg,'k--','LineWidth',1.4); hold on;
plot(baseline.time_s,baseline.pitchAngle_deg,'LineWidth',1.7);
plot(baseline.time_s,baseline.flightPathAngle_deg,':','LineWidth',1.7);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
legend({'pitch command','pitch angle','flight-path angle'},'Location','best');
title('Pitch moves before the flight path');
subplot(2,1,2);
plot(baseline.time_s,baseline.pitchControlCommand_deg,'LineWidth',1.7);
grid on; xlabel('Time (s)'); ylabel('Equivalent pitch-control command (deg)');
title('Inner-loop control effect remains within its 20 deg envelope');

%% Lever 1 - reset pitch dynamics and sweep only altitude-to-pitch gain
altitudeGainSweep_rad_per_m=[0 0.002 0.004 0.006 0.008];
gainAltitude_m=zeros(numel(altitudeGainSweep_rad_per_m),baseline.sampleCount);
gainErrorAtFiveSeconds_m=zeros(size(altitudeGainSweep_rad_per_m));
gainOvershoot_m=zeros(size(altitudeGainSweep_rad_per_m));
gainPitchSaturationFraction=zeros(size(altitudeGainSweep_rad_per_m));
for k=1:numel(altitudeGainSweep_rad_per_m)
    sample=model(altitudeGainSweep_rad_per_m(k),2.4,1);
    gainAltitude_m(k,:)=sample.altitude_m;
    gainErrorAtFiveSeconds_m(k)=sample.altitudeError_m(fiveSecondIndex);
    gainOvershoot_m(k)=sample.peakAltitudeOvershoot_m;
    gainPitchSaturationFraction(k)=sample.pitchCommandSaturationFraction;
    assert(sample.pitchNaturalFrequency_radps== ...
        baseline.pitchNaturalFrequency_radps && ...
        sample.altitudeFeedbackSign==1 && ...
        sample.sampleCount==baseline.sampleCount, ...
        'The altitude-gain sweep must preserve pitch dynamics, sign, and resources.');
end

%% Changed view - gain trades early error for overshoot and saturation
figure('Name','P13 altitude gain sweep');
subplot(2,2,1);
plot(baseline.time_s,gainAltitude_m,'LineWidth',1.25); hold on;
plot(baseline.time_s,baseline.altitudeCommand_m,'k--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Geometric altitude (m)');
gainLegend=cellstr(compose('K_h %.3f rad/m', ...
    altitudeGainSweep_rad_per_m));
gainLegend{end+1}='command';
legend(gainLegend,'Location','best');
title('Higher outer gain captures earlier');
subplot(2,2,2);
plot(altitudeGainSweep_rad_per_m,gainErrorAtFiveSeconds_m,'o-', ...
    'LineWidth',1.5);
grid on; xlabel('Altitude-to-pitch gain (rad/m)');
ylabel('Altitude error at 5 s (m)');
title('Early error falls as gain rises');
subplot(2,2,3);
plot(altitudeGainSweep_rad_per_m,gainOvershoot_m,'s-', ...
    'LineWidth',1.5);
grid on; xlabel('Altitude-to-pitch gain (rad/m)');
ylabel('Altitude overshoot (m)');
title('Path lag can carry altitude past target');
subplot(2,2,4);
plot(altitudeGainSweep_rad_per_m, ...
    100*gainPitchSaturationFraction,'d-','LineWidth',1.4);
grid on; xlabel('Altitude-to-pitch gain (rad/m)');
ylabel('Pitch-command saturation (%)');
title('Authority limits the benefit of more gain');
assert(all(diff(gainErrorAtFiveSeconds_m)<0) && ...
    gainOvershoot_m(1)==0 && gainOvershoot_m(end)>gainOvershoot_m(3) && ...
    gainPitchSaturationFraction(1)==0 && ...
    gainPitchSaturationFraction(end)>0, ...
    'Outer gain must reduce early error while exposing overshoot and saturation.');

%% Read and explain lever 1
% K_h converts metres of altitude error into radians of pitch command. More
% gain asks for a steeper pitch sooner, but the 10 deg pitch-command envelope
% clips that request and the flight-path lag can carry altitude past target.
% Gain is not free performance: it trades error, overshoot, and authority.
disp(['Mechanism 1: outer gain converts altitude error to pitch demand; ' ...
    'path lag and the pitch envelope create the visible tradeoff.']);

%% Lever 2 - reset outer gain and sweep only inner-loop natural frequency
pitchFrequencySweep_radps=[1.2 1.8 2.4 3.0 3.6];
frequencyPitch_deg=zeros(numel(pitchFrequencySweep_radps),baseline.sampleCount);
frequencyPitchAtOnePointFive_deg=zeros(size(pitchFrequencySweep_radps));
frequencyTrackingRMS_deg=zeros(size(pitchFrequencySweep_radps));
frequencyPeakControl_deg=zeros(size(pitchFrequencySweep_radps));
onePointFiveIndex=find(baseline.time_s>=1.5,1,'first');
for k=1:numel(pitchFrequencySweep_radps)
    sample=model(0.004,pitchFrequencySweep_radps(k),1);
    frequencyPitch_deg(k,:)=sample.pitchAngle_deg;
    frequencyPitchAtOnePointFive_deg(k)= ...
        sample.pitchAngle_deg(onePointFiveIndex);
    frequencyTrackingRMS_deg(k)=sample.pitchTrackingRMS_deg;
    frequencyPeakControl_deg(k)=sample.peakPitchControlCommand_deg;
    assert(sample.altitudeGain_rad_per_m==baseline.altitudeGain_rad_per_m && ...
        sample.altitudeFeedbackSign==1 && ...
        isequal(sample.altitudeCommand_m,baseline.altitudeCommand_m), ...
        'The pitch-frequency sweep must preserve outer gain, sign, and command.');
end

%% Changed view - faster pitch tracking demands more control effect
figure('Name','P13 pitch natural frequency sweep');
subplot(1,3,1);
plot(baseline.time_s,frequencyPitch_deg,'LineWidth',1.25);
grid on; xlabel('Time (s)'); ylabel('Pitch angle (deg)');
legend(compose('omega_n %.1f rad/s',pitchFrequencySweep_radps), ...
    'Location','best');
title('Inner-loop pitch response');
subplot(1,3,2);
plot(pitchFrequencySweep_radps,frequencyTrackingRMS_deg,'o-', ...
    'LineWidth',1.5);
grid on; xlabel('Pitch natural frequency (rad/s)');
ylabel('Pitch tracking RMS (deg)');
title('Faster inner loop tracks more closely');
subplot(1,3,3);
plot(pitchFrequencySweep_radps,frequencyPeakControl_deg,'s-', ...
    'LineWidth',1.5);
grid on; xlabel('Pitch natural frequency (rad/s)');
ylabel('Peak pitch-control command (deg)');
title('Faster tracking spends more authority');
assert(all(diff(frequencyPitchAtOnePointFive_deg)>0) && ...
    all(diff(frequencyTrackingRMS_deg)<0) && ...
    all(diff(frequencyPeakControl_deg)>0), ...
    'Pitch frequency must speed early pitch and trade tracking for control demand.');

%% Read and explain lever 2
% The inner-loop gain schedule places the unsaturated pitch natural
% frequency while holding damping ratio fixed. Increasing omega_n makes
% pitch react sooner and lowers command-tracking RMS, but the equivalent
% pitch-control peak rises. The flight path still lags pitch because gamma has
% its own declared response time.
disp(['Mechanism 2: higher pitch natural frequency improves inner tracking ' ...
    'by demanding a larger, earlier control effect; gamma still lags theta.']);

%% Limiting case - zero outer gain opens the altitude loop
openAltitudeLoop=model(0,2.4,1);
assert(all(openAltitudeLoop.altitude_m== ...
    openAltitudeLoop.initialAltitude_m) && ...
    all(openAltitudeLoop.pitchCommand_deg==0) && ...
    all(openAltitudeLoop.pitchAngle_deg==0) && ...
    all(openAltitudeLoop.flightPathAngle_deg==0) && ...
    all(openAltitudeLoop.pitchControlCommand_deg==0) && ...
    openAltitudeLoop.finalAltitudeError_m==openAltitudeLoop.altitudeStep_m, ...
    'Zero altitude gain must leave the step uncorrected with every state at trim.');

%% Deliberately broken case - reverse only the altitude/Down feedback sign
broken=model(0.004,2.4,-1);
figure('Name','P13 broken altitude feedback sign');
subplot(1,2,1);
plot(baseline.time_s,baseline.altitudeCommand_m,'k--','LineWidth',1.3); hold on;
plot(baseline.time_s,baseline.altitude_m,'LineWidth',1.7);
plot(broken.time_s,broken.altitude_m,':','LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Geometric altitude, h=-Down (m)');
legend({'command','correct sign','broken sign'},'Location','best');
title('Wrong sign descends after an upward command');
subplot(1,2,2);
plot(baseline.time_s,baseline.altitudeError_m,'LineWidth',1.7); hold on;
plot(broken.time_s,broken.altitudeError_m,':','LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Altitude error, h_c-h (m)');
legend({'correct sign','broken sign'},'Location','best');
title('Positive feedback grows the error');
assert(isequal(broken.time_s,baseline.time_s) && ...
    broken.altitudeGain_rad_per_m==baseline.altitudeGain_rad_per_m && ...
    broken.pitchNaturalFrequency_radps== ...
    baseline.pitchNaturalFrequency_radps && ...
    isequal(broken.altitude_m(1:commandIndex), ...
    baseline.altitude_m(1:commandIndex)) && ...
    broken.pitchCommand_deg(commandIndex)<0 && ...
    broken.altitude_m(end)<750 && broken.finalAltitudeError_m>300 && ...
    broken.pitchCommandSaturationFraction>0.8, ...
    ['The broken case must isolate a sign reversal and show finite fixed-horizon ' ...
    'divergence with bounded commands.']);
fprintf(['Broken symptom: an upward %.1f m command drives altitude to %.1f m, ' ...
    'leaves %.1f m error, and pins pitch command for %.1f%% of samples.\n'], ...
    broken.altitudeStep_m,broken.altitude_m(end), ...
    broken.finalAltitudeError_m,100*broken.pitchCommandSaturationFraction);

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach-back: trace altitude error through both loops, distinguish ' ...
    'pitch from flight-path angle, name both gain tradeoffs, and diagnose ' ...
    'the reversed Down/altitude feedback sign.']);
