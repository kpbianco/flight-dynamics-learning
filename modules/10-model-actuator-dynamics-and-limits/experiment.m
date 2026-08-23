%% P10 - Model Actuator Dynamics and Limits
% Guiding question:
% What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P10 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - put a transparent actuator between command and P09 moment
% P09 applied body moments directly. A real command first passes through a
% servo with lag, a maximum travel rate, and a mechanical position stop.
% Here delta is a generic signed control-surface deflection in degrees:
%
% delta_dot_raw = (delta_command_limited-delta)/tau
% delta_dot     = clip(delta_dot_raw,+/-rate_limit)
% delta         = clip(delta,+/-position_limit)
%
% The conceptual body-y moment ledger is M_y=80 delta N*m. It has the units
% and axis meaning of P09's internal applied moment, but P09's public model
% accepts scalar pulse scales, not this history; no adapter or closed-loop
% controller is implied.
disp('P09 consumed body moment directly; P10 exposes the actuator between command and delivered moment.');
disp(['Predict once: after a command reverses from beyond the positive hard stop ' ...
    'to beyond the negative stop, which limit controls the first part of the motion?']);

%% Baseline - one fixed command schedule and declared actuator
baseline=model(0.18,45);
fprintf(['Baseline inputs: tau %.2f s, rate limit %.1f deg/s, ' ...
    'position limit +/-%.1f deg, sample time %.3f s, %d samples.\n'], ...
    baseline.timeConstant_s,baseline.rateLimit_deg_s, ...
    baseline.positionLimit_deg,baseline.sampleTime_s,baseline.sampleCount);
fprintf(['Observable response: 90%% of the feasible +%.1f deg target in ' ...
    '%.2f s after the positive step, ' ...
    'reversal zero crossing %.2f s after reversal, peak rate %.1f deg/s.\n'], ...
    baseline.positionLimit_deg,baseline.positiveNinetyResponseTime_s, ...
    baseline.reversalZeroCrossingDelay_s,baseline.peakRate_deg_s);
fprintf(['Limit ledger: position request limited for %.2f s, rate limited for %.2f s, ' ...
    'peak delivered pitch moment %.1f N*m.\n'], ...
    baseline.infeasibleCommandDuration_s,baseline.rateLimitedDuration_s, ...
    baseline.peakDeliveredPitchMoment_Nm);
assert(max(baseline.positionLimitExcess_deg)==0 && ...
    max(baseline.rateLimitExcess_deg_s)<1e-10 && ...
    max(abs(baseline.kinematicClosureResidual_deg))<1e-12, ...
    'The baseline must obey its position, rate, and kinematic invariants.');

%% Baseline view 1 - separate requested, feasible, and delivered deflection
figure('Name','P10 baseline command and deflection');
plot(baseline.time_s,baseline.command_deg,'k:','LineWidth',1.5); hold on;
plot(baseline.time_s,baseline.limitedCommand_deg,'--','LineWidth',1.5);
plot(baseline.time_s,baseline.deflection_deg,'LineWidth',1.8);
plot(baseline.time_s,baseline.positionLimit_deg* ...
    ones(size(baseline.time_s)),'r-.');
plot(baseline.time_s,-baseline.positionLimit_deg* ...
    ones(size(baseline.time_s)),'r-.');
grid on; xlabel('Time (s)'); ylabel('Control-surface deflection (deg)');
legend({'requested command','position-limited command','delivered deflection', ...
    '+ position stop','- position stop'},'Location','best');
title('Command is not delivered deflection');

%% Baseline view 2 - rate saturation changes moment delivery
figure('Name','P10 baseline rate and moment ledger');
subplot(1,2,1);
plot(baseline.time_s,baseline.lagRateDemand_deg_s,'--','LineWidth',1.3); hold on;
plot(baseline.time_s,baseline.actualRate_deg_s,'LineWidth',1.7);
plot(baseline.time_s,baseline.rateLimit_deg_s* ...
    ones(size(baseline.time_s)),'r-.');
plot(baseline.time_s,-baseline.rateLimit_deg_s* ...
    ones(size(baseline.time_s)),'r-.');
grid on; xlabel('Time (s)'); ylabel('Actuator rate (deg/s)');
legend({'lag rate demand','delivered rate','+ rate limit','- rate limit'}, ...
    'Location','best');
title('Rate clips before position updates');
subplot(1,2,2);
plot(baseline.time_s,baseline.requestedPitchMoment_Nm,'k:','LineWidth',1.5); hold on;
plot(baseline.time_s,baseline.feasiblePitchMoment_Nm,'--','LineWidth',1.5);
plot(baseline.time_s,baseline.deliveredPitchMoment_Nm,'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Pitch moment M_y (N*m)');
legend({'requested','within hard stop','delivered'},'Location','best');
title('Conceptual body-y moment boundary');

%% Lever 1 - reset the rate limit and sweep only time constant
timeConstantSweep_s=[0.08 0.12 0.18 0.28 0.40];
timeConstantDeflection_deg=zeros(numel(timeConstantSweep_s),baseline.sampleCount);
timeConstantRmsError_deg=zeros(size(timeConstantSweep_s));
timeConstantNinetyTime_s=zeros(size(timeConstantSweep_s));
for k=1:numel(timeConstantSweep_s)
    sample=model(timeConstantSweep_s(k),45);
    timeConstantDeflection_deg(k,:)=sample.deflection_deg;
    timeConstantRmsError_deg(k)=sample.rmsFeasibleTrackingError_deg;
    timeConstantNinetyTime_s(k)=sample.positiveNinetyResponseTime_s;
    assert(isequal(sample.command_deg,baseline.command_deg) && ...
        isequal(sample.limitedCommand_deg,baseline.limitedCommand_deg) && ...
        sample.rateLimit_deg_s==45 && ...
        max(sample.positionLimitExcess_deg)==0 && ...
        max(sample.rateLimitExcess_deg_s)<1e-10, ...
        'The time-constant sweep must preserve commands and both limits.');
end

%% Changed view - slower lag raises feasible tracking error
figure('Name','P10 time-constant sweep');
subplot(1,3,1);
plot(baseline.time_s,timeConstantDeflection_deg,'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Delivered deflection (deg)');
legend(compose('tau %.2f s',timeConstantSweep_s),'Location','best');
title('Only actuator lag changes');
subplot(1,3,2);
plot(timeConstantSweep_s,timeConstantNinetyTime_s,'o-','LineWidth',1.4);
grid on; xlabel('Time constant tau (s)');
ylabel('90% feasible-target response time (s)');
title('More lag delays authority');
subplot(1,3,3);
plot(timeConstantSweep_s,timeConstantRmsError_deg,'s-','LineWidth',1.4);
grid on; xlabel('Time constant tau (s)');
ylabel('Feasible-command RMS error (deg)');
title('More lag raises tracking error');
assert(all(diff(timeConstantNinetyTime_s)>0) && ...
    all(diff(timeConstantRmsError_deg)>0), ...
    'A larger time constant must slow the response and increase error.');

%% Read and explain lever 1
% Tau divides the remaining feasible position error. For the same remaining
% error, a larger tau asks for less raw rate. Sweep trajectories do not retain
% the same error at the same time, so the trustworthy comparisons are the
% observed response time and tracking error, not an assumed pointwise rate
% ordering. Neither physical limit moved.
disp(['Mechanism 1: for the same remaining error, increasing tau reduces raw ' ...
    'lag demand; across the sweep it delays delivery and raises tracking error.']);

%% Lever 2 - reset tau and sweep only the rate limit
rateLimitSweep_deg_s=[20 30 45 60 80];
rateLimitDeflection_deg=zeros(numel(rateLimitSweep_deg_s),baseline.sampleCount);
rateLimitRmsError_deg=zeros(size(rateLimitSweep_deg_s));
rateLimitNinetyTime_s=zeros(size(rateLimitSweep_deg_s));
peakRateSweep_deg_s=zeros(size(rateLimitSweep_deg_s));
for k=1:numel(rateLimitSweep_deg_s)
    sample=model(0.18,rateLimitSweep_deg_s(k));
    rateLimitDeflection_deg(k,:)=sample.deflection_deg;
    rateLimitRmsError_deg(k)=sample.rmsFeasibleTrackingError_deg;
    rateLimitNinetyTime_s(k)=sample.positiveNinetyResponseTime_s;
    peakRateSweep_deg_s(k)=sample.peakRate_deg_s;
    assert(isequal(sample.command_deg,baseline.command_deg) && ...
        isequal(sample.limitedCommand_deg,baseline.limitedCommand_deg) && ...
        sample.timeConstant_s==0.18 && ...
        max(sample.positionLimitExcess_deg)==0 && ...
        max(sample.rateLimitExcess_deg_s)<1e-10, ...
        'The rate-limit sweep must preserve command, hard stop, and lag.');
end

%% Changed view - rate authority sets reversal speed
figure('Name','P10 rate-limit sweep');
subplot(2,2,1);
plot(baseline.time_s,rateLimitDeflection_deg,'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Delivered deflection (deg)');
legend(compose('rate %.0f deg/s',rateLimitSweep_deg_s),'Location','best');
title('Only rate authority changes');
subplot(2,2,2);
plot(rateLimitSweep_deg_s,rateLimitNinetyTime_s,'o-','LineWidth',1.4);
grid on; xlabel('Rate limit (deg/s)');
ylabel('90% feasible-target response time (s)');
title('More rate shortens response');
subplot(2,2,3);
plot(rateLimitSweep_deg_s,rateLimitRmsError_deg,'s-','LineWidth',1.4);
grid on; xlabel('Rate limit (deg/s)');
ylabel('Feasible-command RMS error (deg)');
title('More rate lowers tracking error');
subplot(2,2,4);
plot(rateLimitSweep_deg_s,peakRateSweep_deg_s,'^-','LineWidth',1.4);
grid on; xlabel('Rate limit (deg/s)');
ylabel('Peak delivered rate (deg/s)');
title('The schedule reaches each rate stop');
assert(all(diff(rateLimitNinetyTime_s)<0) && ...
    all(diff(rateLimitRmsError_deg)<0) && ...
    all(diff(peakRateSweep_deg_s)>0), ...
    'More rate authority must shorten response and reduce feasible error.');

%% Read and explain lever 2
% The reversal initially asks for much more than the available rate. During
% that interval, tau no longer sets delivered rate: the explicit rate clip
% does. Once the remaining error shrinks, the first-order lag takes over.
disp('Mechanism 2: during a large reversal the rate stop, not the raw lag demand, sets delivered motion.');

%% Limiting case - enough rate authority exposes the pure first-order recurrence
lagOnly=model(0.50,120);
assert(~any(lagOnly.rateLimitActive) && ...
    max(abs(lagOnly.kinematicClosureResidual_deg))<1e-12 && ...
    max(lagOnly.positionLimitExcess_deg)==0, ...
    'At tau 0.50 s and 120 deg/s, only lag and the command hard stop remain.');
fprintf(['Lag-only accepted corner: no rate-limited samples; final deflection ' ...
    '%.3f deg follows the visible Euler recurrence.\n'],lagOnly.finalDeflection_deg);

%% Deliberately broken case - omit the one declared position envelope
figure('Name','P10 broken omitted position envelope');
subplot(1,2,1);
plot(baseline.time_s,baseline.command_deg,'k:','LineWidth',1.4); hold on;
plot(baseline.time_s,baseline.deflection_deg,'LineWidth',1.7);
plot(baseline.time_s,baseline.brokenDeflection_deg,'--','LineWidth',1.7);
plot(baseline.time_s,baseline.positionLimit_deg* ...
    ones(size(baseline.time_s)),'r-.');
plot(baseline.time_s,-baseline.positionLimit_deg* ...
    ones(size(baseline.time_s)),'r-.');
grid on; xlabel('Time (s)'); ylabel('Deflection (deg)');
legend({'command','complete actuator','broken actuator', ...
    '+ hard stop','- hard stop'},'Location','best');
title('Smooth output can still cross an envelope');
subplot(1,2,2);
plot(baseline.time_s,baseline.deliveredPitchMoment_Nm,'LineWidth',1.7); hold on;
plot(baseline.time_s,baseline.brokenDeliveredPitchMoment_Nm,'--','LineWidth',1.7);
grid on; xlabel('Time (s)'); ylabel('Delivered pitch moment (N*m)');
legend({'complete','position envelope omitted'},'Location','best');
title('Broken model invents moment authority');
assert(baseline.brokenMaximumPositionExcess_deg>9 && ...
    baseline.brokenPeakDeliveredPitchMoment_Nm> ...
    baseline.positionLimit_deg*baseline.momentPerDeflection_Nm_per_deg && ...
    max(baseline.brokenRateLimitExcess_deg_s)<1e-10, ...
    'The broken case must violate only the position envelope, not the rate limit.');
fprintf(['Broken symptom: %.3f deg beyond the declared hard stop and %.1f N*m ' ...
    'of invented peak authority.\n'],baseline.brokenMaximumPositionExcess_deg, ...
    baseline.brokenPeakMomentExcess_Nm);

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach-back: trace command through position limiting, lag, rate limiting, ' ...
    'delivered deflection, and the conceptual body-y moment ledger; then diagnose ' ...
    'the broken hard stop.']);
