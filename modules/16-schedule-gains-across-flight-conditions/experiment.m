%% P16 - Schedule Gains Across Flight Conditions
% Guiding question:
% What inputs, observable effects, and failure modes matter when you schedule Gains Across Flight Conditions?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P16 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - flight condition changes control effectiveness
% P15 made true airspeed visible instead of treating it as a fixed label.
% P16 freezes one condition per run and adds density, so dynamic pressure is
%
%   qbar = 0.5*rho*V^2.
%
% A transparent roll plant uses b=b_ref*qbar/qbar_ref. The controller looks
% up and linearly interpolates angle and rate gains:
%
%   delta_a = sat(K_phi*(phi_command-phi)-K_p*p,+/-delta_max)
%   phi_dot = p
%   p_dot   = b(qbar)*delta_a
%
% The table is a frozen-condition teaching schedule, not aircraft data.
disp('P16 maps true airspeed and density into dynamic pressure, lookup gains, equivalent aileron, and roll response.');
disp(['Predict once: if dynamic pressure falls but the gains stay fixed, ' ...
    'will the same roll command remain equally damped?']);

%% Baseline - reference condition lands exactly on the center knot
baseline=model(60,0.736115547399152,1);
fixedReference=model(60,0.736115547399152,0);
commandIndex=find(baseline.time_s>=baseline.commandStepTime_s,1,'first');
fprintf(['Baseline: V %.1f m/s, rho %.12f kg/m^3, qbar %.3f Pa, ' ...
    'qbar/qbar_ref %.3f.\n'],baseline.trueAirspeed_mps, ...
    baseline.airDensity_kgpm3,baseline.actualDynamicPressure_Pa, ...
    baseline.actualDynamicPressureRatio);
fprintf(['Lookup K_phi %.3f rad/rad and K_p %.3f s gives effective ' ...
    'omega_n %.3f rad/s and zeta %.3f.\n'],baseline.rollAngleGain, ...
    baseline.rollRateGain_s,baseline.effectiveNaturalFrequency_radps, ...
    baseline.effectiveDampingRatio);
fprintf(['90%% capture %.2f s; settling %.2f s; overshoot %.4f deg; ' ...
    'peak equivalent aileron %.3f deg.\n'], ...
    baseline.timeToNinetyPercent_s,baseline.settlingTime_s, ...
    baseline.peakRollOvershoot_deg, ...
    baseline.peakAbsoluteAileronCommand_deg);
assert(baseline.sampleCount==801 && baseline.intervalCount==800 && ...
    baseline.actualDynamicPressureRatio==1 && ...
    abs(baseline.rollAngleGain-0.48)<1e-14 && ...
    abs(baseline.rollRateGain_s-0.32)<1e-14 && ...
    abs(baseline.aileronCommand_deg(commandIndex)-4.8)<1e-12 && ...
    abs(baseline.timeToNinetyPercent_s-1.23)<1e-12 && ...
    abs(baseline.settlingTime_s-1.55)<1e-12 && ...
    abs(baseline.peakRollOvershoot_deg-0.1615456758015)<1e-10 && ...
    isequal(baseline.rollAngle_deg,fixedReference.rollAngle_deg), ...
    'The reference condition must reproduce the center-knot response.');

%% Baseline view 1 - follow roll command, angle, and error
figure('Name','P16 baseline roll response');
subplot(2,1,1);
plot(baseline.time_s,baseline.rollCommand_deg,'k--','LineWidth',1.4); hold on;
plot(baseline.time_s,baseline.rollAngle_deg,'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Roll angle (deg)');
legend({'command','scheduled response'},'Location','best');
title('Reference dynamic pressure uses the center gain knot');
subplot(2,1,2);
plot(baseline.time_s,baseline.rollError_deg,'LineWidth',1.8); hold on;
yline(baseline.settlingTolerance_deg,'k:');
yline(-baseline.settlingTolerance_deg,'k:');
grid on; xlabel('Time (s)'); ylabel('Roll error (deg)');
title('Command-minus-angle error enters the 2% band');

%% Baseline view 2 - table gains become bounded control demand
figure('Name','P16 baseline gain lookup and control');
subplot(2,1,1);
plot(baseline.dynamicPressureRatioKnots,baseline.rollAngleGainTable, ...
    'o-','LineWidth',1.6); hold on;
plot(baseline.dynamicPressureRatioKnots,baseline.rollRateGainTable_s, ...
    's-','LineWidth',1.6);
plot(baseline.lookupDynamicPressureRatio,baseline.rollAngleGain, ...
    'ko','MarkerFaceColor','k');
grid on; xlabel('Lookup dynamic-pressure ratio qbar/qbar_{ref}');
ylabel('Scheduled gain (rad/rad or s)');
legend({'K_phi table','K_p table','current K_phi'},'Location','best');
title('Manual linear interpolation between five ordered knots');
subplot(2,1,2);
plot(baseline.time_s,baseline.aileronCommand_deg,'LineWidth',1.8); hold on;
yline(baseline.aileronCommandLimit_deg,'r--');
yline(-baseline.aileronCommandLimit_deg,'r--');
grid on; xlabel('Time (s)'); ylabel('Equivalent aileron command (deg)');
title('Interpolated gains stay inside the declared command envelope');

%% Lever 1 - hold density fixed and sweep only true airspeed
airspeedSweep_mps=[45 52.5 60 67.5 72];
scheduledRollByAirspeed_deg=zeros(numel(airspeedSweep_mps), ...
    baseline.sampleCount);
scheduledAirspeedSettling_s=zeros(size(airspeedSweep_mps));
fixedAirspeedSettling_s=zeros(size(airspeedSweep_mps));
scheduledAirspeedOvershoot_deg=zeros(size(airspeedSweep_mps));
fixedAirspeedOvershoot_deg=zeros(size(airspeedSweep_mps));
scheduledAirspeedPeakAileron_deg=zeros(size(airspeedSweep_mps));
for k=1:numel(airspeedSweep_mps)
    scheduled=model(airspeedSweep_mps(k), ...
        baseline.referenceAirDensity_kgpm3,1);
    fixed=model(airspeedSweep_mps(k), ...
        baseline.referenceAirDensity_kgpm3,0);
    scheduledRollByAirspeed_deg(k,:)=scheduled.rollAngle_deg;
    scheduledAirspeedSettling_s(k)=scheduled.settlingTime_s;
    fixedAirspeedSettling_s(k)=fixed.settlingTime_s;
    scheduledAirspeedOvershoot_deg(k)=scheduled.peakRollOvershoot_deg;
    fixedAirspeedOvershoot_deg(k)=fixed.peakRollOvershoot_deg;
    scheduledAirspeedPeakAileron_deg(k)= ...
        scheduled.peakAbsoluteAileronCommand_deg;
    assert(scheduled.airDensity_kgpm3== ...
        baseline.referenceAirDensity_kgpm3 && scheduled.scheduleMode==1 && ...
        fixed.airDensity_kgpm3==baseline.referenceAirDensity_kgpm3 && ...
        fixed.scheduleMode==0 && isequal(scheduled.time_s,baseline.time_s), ...
        'The airspeed sweep must hold density, modes, and grid fixed.');
end

%% Changed view - scheduling preserves response while control demand changes
figure('Name','P16 true airspeed sweep');
subplot(1,3,1);
plot(baseline.time_s,scheduledRollByAirspeed_deg,'LineWidth',1.2); hold on;
plot(baseline.time_s,baseline.rollCommand_deg,'k--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Roll angle (deg)');
airspeedLegend=cellstr(compose('V %.1f m/s',airspeedSweep_mps));
airspeedLegend{end+1}='command';
legend(airspeedLegend,'Location','best');
title('Scheduled responses stay nearly coincident');
subplot(1,3,2);
plot(airspeedSweep_mps,scheduledAirspeedSettling_s,'o-', ...
    'LineWidth',1.5); hold on;
plot(airspeedSweep_mps,fixedAirspeedSettling_s,'s--','LineWidth',1.5);
grid on; xlabel('True airspeed (m/s)'); ylabel('2% settling time (s)');
legend({'scheduled','fixed reference gains'},'Location','best');
title('Fixed gains expose condition sensitivity');
subplot(1,3,3);
plot(airspeedSweep_mps,scheduledAirspeedPeakAileron_deg, ...
    'd-','LineWidth',1.5);
grid on; xlabel('True airspeed (m/s)');
ylabel('Peak equivalent aileron (deg)');
title('Less command is needed as qbar rises');
assert(max(scheduledAirspeedSettling_s)- ...
    min(scheduledAirspeedSettling_s)<=0.02 && ...
    max(fixedAirspeedSettling_s)-min(fixedAirspeedSettling_s)>1.5 && ...
    all(diff(scheduledAirspeedPeakAileron_deg)<0), ...
    'Airspeed scheduling must preserve timing and expose changing control demand.');

%% Read and explain lever 1
% At fixed density, qbar grows with V^2 and the plant becomes more
% effective. The table reduces both gains as qbar rises. Scheduled timing
% therefore stays near the target while peak equivalent aileron falls.
% Fixed gains cannot make that compensation; faster is not automatically
% better because its effective damping and bandwidth also move.
disp(['Mechanism 1: V changes qbar and plant effectiveness; inverse-like ' ...
    'table gains keep the declared roll response near its target.']);

%% Lever 2 - reset airspeed and sweep only density at exact table knots
densityRatioSweep=[0.5 0.75 1 1.25 1.5];
densitySweep_kgpm3=baseline.referenceAirDensity_kgpm3*densityRatioSweep;
scheduledRollByDensity_deg=zeros(numel(densityRatioSweep), ...
    baseline.sampleCount);
densityNaturalFrequency_radps=zeros(size(densityRatioSweep));
densityDampingRatio=zeros(size(densityRatioSweep));
densityPeakAileron_deg=zeros(size(densityRatioSweep));
for k=1:numel(densityRatioSweep)
    scheduled=model(baseline.referenceTrueAirspeed_mps, ...
        densitySweep_kgpm3(k),1);
    scheduledRollByDensity_deg(k,:)=scheduled.rollAngle_deg;
    densityNaturalFrequency_radps(k)= ...
        scheduled.effectiveNaturalFrequency_radps;
    densityDampingRatio(k)=scheduled.effectiveDampingRatio;
    densityPeakAileron_deg(k)=scheduled.peakAbsoluteAileronCommand_deg;
    assert(scheduled.trueAirspeed_mps== ...
        baseline.referenceTrueAirspeed_mps && scheduled.scheduleMode==1 && ...
        abs(scheduled.actualDynamicPressureRatio-densityRatioSweep(k))<1e-12, ...
        'The density sweep must hold airspeed and schedule mode fixed.');
end

%% Changed view - exact knots preserve poles but change required command
figure('Name','P16 air density sweep');
subplot(1,3,1);
plot(baseline.time_s,scheduledRollByDensity_deg,'LineWidth',1.2); hold on;
plot(baseline.time_s,baseline.rollCommand_deg,'k--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Roll angle (deg)');
densityLegend=cellstr(compose('rho/rho_ref %.2f',densityRatioSweep));
densityLegend{end+1}='command';
legend(densityLegend,'Location','best');
title('Knot responses overlay at fixed target poles');
subplot(1,3,2);
plot(densityRatioSweep,densityNaturalFrequency_radps,'o-', ...
    'LineWidth',1.5); hold on;
plot(densityRatioSweep,densityDampingRatio,'s-','LineWidth',1.5);
grid on; xlabel('Density ratio rho/rho_{ref}');
ylabel('Frequency (rad/s) or damping ratio');
legend({'effective omega_n','effective zeta'},'Location','best');
title('Each knot closes to the declared target');
subplot(1,3,3);
plot(densityRatioSweep,densityPeakAileron_deg,'d-', ...
    'LineWidth',1.5);
grid on; xlabel('Density ratio rho/rho_{ref}');
ylabel('Peak equivalent aileron (deg)');
title('Low qbar demands more control angle');
assert(max(max(abs(scheduledRollByDensity_deg- ...
    scheduledRollByDensity_deg(3,:))))<1e-10 && ...
    max(abs(densityNaturalFrequency_radps-2.4))<1e-12 && ...
    max(abs(densityDampingRatio-0.8))<1e-12 && ...
    all(diff(densityPeakAileron_deg)<0), ...
    'Exact density knots must preserve response while command demand falls.');

%% Read and explain lever 2 and the reference limiting case
% At V_ref, density ratio equals dynamic-pressure ratio. Every swept value
% is an exact table knot, so b*K_phi=omega_n^2 and
% b*K_p=2*zeta*omega_n. At qbar/qbar_ref=1, scheduled and fixed gains are
% exactly identical: scheduling has no effect at its reference condition.
assert(isequal(baseline.rollAngle_deg,fixedReference.rollAngle_deg) && ...
    isequal(baseline.aileronCommand_deg, ...
    fixedReference.aileronCommand_deg), ...
    'Scheduled and fixed modes must coincide exactly at the reference knot.');
disp(['Mechanism 2: rho changes qbar independently of V; exact knot gains ' ...
    'preserve target poles, while the reference knot is the fixed-gain limit.']);

%% Deliberately broken - true airspeed is not dynamic pressure
equalDynamicPressureAirspeed_mps=75;
equalDynamicPressureDensity_kgpm3= ...
    baseline.referenceAirDensity_kgpm3*( ...
    baseline.referenceTrueAirspeed_mps/ ...
    equalDynamicPressureAirspeed_mps)^2;
equalQCorrect=model(equalDynamicPressureAirspeed_mps, ...
    equalDynamicPressureDensity_kgpm3,1);
broken=model(equalDynamicPressureAirspeed_mps, ...
    equalDynamicPressureDensity_kgpm3,-1);
fprintf(['Equal-qbar pair: V %.1f m/s and rho %.6f kg/m^3 still gives ' ...
    'qbar %.3f Pa.\n'],equalQCorrect.trueAirspeed_mps, ...
    equalQCorrect.airDensity_kgpm3,equalQCorrect.actualDynamicPressure_Pa);
fprintf(['Correct lookup ratio %.3f; broken TAS-only raw ratio %.4f ' ...
    'clamps to %.3f. Selected K_phi is %.1f%% from the actual-condition ' ...
    'ideal. Broken settling %.2f s and overshoot %.3f deg.\n'], ...
    equalQCorrect.lookupDynamicPressureRatio, ...
    broken.lookupDynamicPressureRatioRaw, ...
    broken.lookupDynamicPressureRatio, ...
    100*broken.rollAngleGainActualConditionMismatchFraction, ...
    broken.settlingTime_s, ...
    broken.peakRollOvershoot_deg);
assert(abs(equalQCorrect.actualDynamicPressure_Pa- ...
    baseline.actualDynamicPressure_Pa)<1e-9 && ...
    max(abs(equalQCorrect.rollAngle_deg-baseline.rollAngle_deg))<1e-12 && ...
    broken.lookupClamped && ...
    abs(broken.lookupDynamicPressureRatioRaw-1.5625)<1e-12 && ...
    abs(broken.rollAngleGainActualConditionMismatchFraction+1/3)<1e-12 && ...
    broken.peakRollOvershoot_deg> ...
    equalQCorrect.peakRollOvershoot_deg+0.5 && ...
    broken.settlingTime_s>equalQCorrect.settlingTime_s+1.4 && ...
    broken.peakAbsoluteAileronCommand_deg< ...
    equalQCorrect.peakAbsoluteAileronCommand_deg, ...
    'The TAS-only schedule must isolate a wrong lookup and recognizable response.');

figure('Name','P16 broken true airspeed lookup');
subplot(2,1,1);
plot(equalQCorrect.time_s,equalQCorrect.rollCommand_deg,'k--', ...
    'LineWidth',1.3); hold on;
plot(equalQCorrect.time_s,equalQCorrect.rollAngle_deg,'LineWidth',1.8);
plot(broken.time_s,broken.rollAngle_deg,'r-.','LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Roll angle (deg)');
legend({'command','correct qbar lookup','broken TAS-only lookup'}, ...
    'Location','best');
title('Same plant qbar, different lookup: broken response overshoots');
subplot(2,1,2);
plot(equalQCorrect.time_s,equalQCorrect.aileronCommand_deg, ...
    'LineWidth',1.8); hold on;
plot(broken.time_s,broken.aileronCommand_deg,'r-.','LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Equivalent aileron command (deg)');
legend({'correct qbar lookup','broken TAS-only lookup'},'Location','best');
title('Omitting density selects gains that are too small');

%% Check and teach back
% The failure is not a change in actual dynamic pressure, plant, command,
% grid, or control limit. Only the scheduling variable is wrong. Run the
% independent checks, then answer the guiding question in two sentences.
clear run_checks;
run_checks;
