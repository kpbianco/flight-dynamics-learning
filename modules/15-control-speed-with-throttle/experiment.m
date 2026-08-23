%% P15 - Control Speed with Throttle
% Guiding question:
% What inputs, observable effects, and failure modes matter when you control Speed with Throttle?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P15 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - speed feedback asks throttle to unbalance thrust and drag
% P14 held heading at a declared fixed 60 m/s. P15 now makes that speed a
% state. A transparent controller adds corrective force to level-flight
% drag feedforward, a first-order throttle response delivers thrust, and the
% thrust-minus-drag residual accelerates the mass:
%
%   e_V       = V_command-V
%   D(V)      = A*V^2+B/V^2
%   T_command = sat(D(V)+s*m*K_V*e_V,0,T_max)
%   delta_dot = (T_command/T_max-delta)/tau_T
%   V_dot     = (T_max*delta-D(V))/m
%
% Correct feedback uses s=+1. The exact drag term is ideal teaching-model
% feedforward, not measured aircraft or propulsion data.
disp('P15 traces speed error through thrust command, throttle lag, force imbalance, and airspeed.');
disp(['Predict once: after a positive speed-command step, does acceleration ' ...
    'change at the same sample as requested throttle or only after delivered throttle moves?']);

%% Baseline - command 60 to 70 m/s with correct feedback
baseline=model(0.15,0.8,1);
commandIndex=find(baseline.time_s>=baseline.commandStepTime_s,1,'first');
onePointFiveIndex=find(baseline.time_s>=1.5,1,'first');
twoSecondIndex=find(baseline.time_s>=2,1,'first');
fiveSecondIndex=find(baseline.time_s>=5,1,'first');
tenSecondIndex=find(baseline.time_s>=10,1,'first');
fprintf(['Baseline: K_V %.3f 1/s, throttle tau %.1f s, speed %.0f to ' ...
    '%.0f m/s at %.1f s.\n'],baseline.speedGain_per_s, ...
    baseline.throttleTimeConstant_s,baseline.initialTrueAirspeed_mps, ...
    baseline.commandedTrueAirspeed_mps,baseline.commandStepTime_s);
fprintf(['Trim drag %.3f N requires %.3f throttle. At command onset, ' ...
    'requested thrust jumps to %.3f N while acceleration remains %.3f m/s^2.\n'], ...
    baseline.initialDrag_N,baseline.trimThrottle, ...
    baseline.thrustCommand_N(commandIndex), ...
    baseline.longitudinalAcceleration_mps2(commandIndex));
fprintf(['At 1.5 s: delivered throttle %.4f and acceleration %.4f m/s^2. ' ...
    'At 5 s: speed %.4f m/s; at 10 s: error %.4f m/s.\n'], ...
    baseline.throttleActual(onePointFiveIndex), ...
    baseline.longitudinalAcceleration_mps2(onePointFiveIndex), ...
    baseline.trueAirspeed_mps(fiveSecondIndex), ...
    baseline.speedError_mps(tenSecondIndex));
fprintf(['Final error %.5f m/s; 90%% capture %.2f s; settling %.2f s; ' ...
    'peak throttle rate %.4f 1/s.\n'],baseline.finalSpeedError_mps, ...
    baseline.timeToNinetyPercent_s,baseline.settlingTime_s, ...
    baseline.peakAbsoluteThrottleRate_per_s);
assert(baseline.sampleCount==1501 && baseline.intervalCount==1500 && ...
    abs(baseline.speedError_mps(commandIndex)-10)<1e-12 && ...
    abs(baseline.thrustCommand_N(commandIndex)- ...
    2626.9521725905024)<1e-9 && ...
    baseline.longitudinalAcceleration_mps2(commandIndex)==0 && ...
    abs(baseline.trueAirspeed_mps(fiveSecondIndex)- ...
    64.07175584861862)<1e-9 && ...
    baseline.finalSpeedError_mps<0.09 && baseline.settledByEnd && ...
    baseline.stallEnvelopeMaintained, ...
    'The baseline must preserve trim, throttle lag, capture, and the speed envelope.');

%% Baseline view 1 - follow speed command, speed, then error
figure('Name','P15 baseline speed capture');
subplot(2,1,1);
plot(baseline.time_s,baseline.speedCommand_mps,'k--','LineWidth',1.4); hold on;
plot(baseline.time_s,baseline.trueAirspeed_mps,'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('True airspeed (m/s)');
legend({'command','response'},'Location','best');
title('Throttle changes airspeed after delivered thrust moves');
subplot(2,1,2);
plot(baseline.time_s,baseline.speedError_mps,'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Speed error (m/s)');
title('Correct feedback contracts command-minus-speed error');

%% Baseline view 2 - requested throttle precedes force and acceleration
figure('Name','P15 baseline throttle and force cascade');
subplot(3,1,1);
plot(baseline.time_s,100*baseline.throttleCommand,'k--','LineWidth',1.4); hold on;
plot(baseline.time_s,100*baseline.throttleActual,'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Throttle (%)');
legend({'command','delivered'},'Location','best');
title('First-order throttle separates request from delivery');
subplot(3,1,2);
plot(baseline.time_s,baseline.thrustActual_N,'LineWidth',1.7); hold on;
plot(baseline.time_s,baseline.drag_N,'LineWidth',1.7);
grid on; xlabel('Time (s)'); ylabel('Force (N)');
legend({'delivered thrust','drag'},'Location','best');
title('Thrust above drag creates positive net force');
subplot(3,1,3);
plot(baseline.time_s,baseline.longitudinalAcceleration_mps2, ...
    'LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Longitudinal acceleration (m/s^2)');
title('Force imbalance divided by mass changes speed');

%% Lever 1 - reset throttle lag and sweep only speed gain
speedGainSweep_per_s=[0 0.075 0.15 0.225 0.3];
gainSpeed_mps=zeros(numel(speedGainSweep_per_s),baseline.sampleCount);
gainSpeedAtFive_mps=zeros(size(speedGainSweep_per_s));
gainErrorAtTen_mps=zeros(size(speedGainSweep_per_s));
gainCaptureTime_s=zeros(size(speedGainSweep_per_s));
gainPeakThrottle=zeros(size(speedGainSweep_per_s));
gainSaturationFraction=zeros(size(speedGainSweep_per_s));
for k=1:numel(speedGainSweep_per_s)
    sample=model(speedGainSweep_per_s(k),0.8,1);
    gainSpeed_mps(k,:)=sample.trueAirspeed_mps;
    gainSpeedAtFive_mps(k)=sample.speedAtFiveSeconds_mps;
    gainErrorAtTen_mps(k)=sample.speedErrorAtTenSeconds_mps;
    gainCaptureTime_s(k)=sample.timeToNinetyPercent_s;
    gainPeakThrottle(k)=sample.peakThrottleActual;
    gainSaturationFraction(k)=sample.thrustCommandSaturationFraction;
    assert(sample.throttleTimeConstant_s== ...
        baseline.throttleTimeConstant_s && ...
        sample.speedFeedbackSign==1 && ...
        isequal(sample.time_s,baseline.time_s) && ...
        isequal(sample.speedCommand_mps,baseline.speedCommand_mps) && ...
        sample.mass_kg==baseline.mass_kg, ...
        'The speed-gain sweep must preserve lag, sign, grid, command, and mass.');
end

%% Changed view - gain trades capture for throttle authority
figure('Name','P15 speed gain sweep');
subplot(2,2,1);
plot(baseline.time_s,gainSpeed_mps,'LineWidth',1.2); hold on;
plot(baseline.time_s,baseline.speedCommand_mps,'k--','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('True airspeed (m/s)');
gainLegend=cellstr(compose('K_V %.3f 1/s',speedGainSweep_per_s));
gainLegend{end+1}='speed command';
legend(gainLegend,'Location','best');
title('More gain asks for corrective force sooner');
subplot(2,2,2);
plot(speedGainSweep_per_s,gainErrorAtTen_mps,'o-','LineWidth',1.5);
grid on; xlabel('Speed gain (1/s)');
ylabel('Speed error at 10 s (m/s)');
title('Early error falls as gain rises');
subplot(2,2,3);
plot(speedGainSweep_per_s,100*gainPeakThrottle,'s-','LineWidth',1.5);
grid on; xlabel('Speed gain (1/s)'); ylabel('Peak delivered throttle (%)');
title('Faster capture uses more throttle authority');
subplot(2,2,4);
plot(speedGainSweep_per_s,100*gainSaturationFraction, ...
    'd-','LineWidth',1.5);
grid on; xlabel('Speed gain (1/s)');
ylabel('Thrust-command saturation (%)');
title('The highest gain reaches the thrust limit');
assert(all(diff(gainSpeedAtFive_mps)>0) && ...
    all(diff(gainErrorAtTen_mps)<0) && ...
    all(diff(gainCaptureTime_s(2:end))<0) && ...
    all(diff(gainPeakThrottle)>0) && ...
    gainSaturationFraction(3)==0 && gainSaturationFraction(end)>0, ...
    'Speed gain must expose capture, throttle-demand, and authority tradeoffs.');

%% Read and explain lever 1
% K_V maps m/s of speed error into m/s^2 of desired acceleration. Mass
% turns that request into corrective force. More gain contracts early error
% and demands more throttle, but the 4000 N thrust envelope caps authority.
% It does not change the fixed throttle time constant or drag equation.
disp(['Mechanism 1: speed gain converts error to corrective force; ' ...
    'the thrust envelope limits the available acceleration request.']);

%% Lever 2 - reset gain and sweep only throttle response time
throttleTimeSweep_s=[0.2 0.5 0.8 1.1 1.4];
timeConstantThrottle=zeros(numel(throttleTimeSweep_s),baseline.sampleCount);
timeConstantSpeedAtTwo_mps=zeros(size(throttleTimeSweep_s));
timeConstantTrackingRMS=zeros(size(throttleTimeSweep_s));
timeConstantPeakRate_per_s=zeros(size(throttleTimeSweep_s));
for k=1:numel(throttleTimeSweep_s)
    sample=model(0.15,throttleTimeSweep_s(k),1);
    timeConstantThrottle(k,:)=sample.throttleActual;
    timeConstantSpeedAtTwo_mps(k)= ...
        sample.trueAirspeed_mps(twoSecondIndex);
    timeConstantTrackingRMS(k)=sample.throttleTrackingRMS;
    timeConstantPeakRate_per_s(k)= ...
        sample.peakAbsoluteThrottleRate_per_s;
    assert(sample.speedGain_per_s==baseline.speedGain_per_s && ...
        sample.speedFeedbackSign==1 && ...
        isequal(sample.speedCommand_mps,baseline.speedCommand_mps) && ...
        sample.maximumThrust_N==baseline.maximumThrust_N && ...
        sample.trimThrottle==baseline.trimThrottle, ...
        'The throttle-time sweep must preserve gain, sign, command, thrust, and trim.');
end

%% Changed view - faster throttle tracking costs throttle rate
figure('Name','P15 throttle time constant sweep');
subplot(1,3,1);
plot(baseline.time_s,100*timeConstantThrottle,'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Delivered throttle (%)');
legend(compose('tau_T %.1f s',throttleTimeSweep_s),'Location','best');
title('Throttle response to the same controller');
subplot(1,3,2);
plot(throttleTimeSweep_s,timeConstantTrackingRMS,'o-','LineWidth',1.5);
grid on; xlabel('Throttle time constant (s)');
ylabel('Throttle tracking RMS (fraction)');
title('More lag increases request-delivery mismatch');
subplot(1,3,3);
plot(throttleTimeSweep_s,timeConstantPeakRate_per_s, ...
    's-','LineWidth',1.5);
grid on; xlabel('Throttle time constant (s)');
ylabel('Peak throttle rate (1/s)');
title('Faster delivery requires more throttle rate');
assert(all(diff(timeConstantSpeedAtTwo_mps)<0) && ...
    all(diff(timeConstantTrackingRMS)>0) && ...
    all(diff(timeConstantPeakRate_per_s)<0), ...
    'Throttle lag must trade early speed/tracking for throttle-rate demand.');

%% Read and explain lever 2
% tau_T changes how quickly delivered throttle follows a request computed
% by the same controller law. Every sweep case has an identical onset
% request because its state is still at trim; subsequent requests differ
% through feedback once the speed histories diverge. A smaller time
% constant produces more speed by 2 s and lowers tracking RMS, but it
% requires a larger throttle rate. It is a normalized first-order teaching
% response, not an identified engine or P10 adapter.
disp(['Mechanism 2: a smaller time constant follows the identical onset ' ...
    'request sooner; subsequent requests differ through feedback, and ' ...
    'faster delivery costs normalized throttle rate.']);

%% Limiting case - zero speed gain preserves the exact initial trim
openSpeedLoop=model(0,0.8,1);
assert(all(openSpeedLoop.trueAirspeed_mps== ...
    openSpeedLoop.initialTrueAirspeed_mps) && ...
    all(openSpeedLoop.throttleCommand==openSpeedLoop.trimThrottle) && ...
    all(openSpeedLoop.throttleActual==openSpeedLoop.trimThrottle) && ...
    all(openSpeedLoop.throttleRate_per_s==0) && ...
    max(abs(openSpeedLoop.netForwardForce_N))<1e-12 && ...
    max(abs(openSpeedLoop.longitudinalAcceleration_mps2))<1e-15 && ...
    abs(openSpeedLoop.finalSpeedError_mps-10)<1e-12 && ...
    ~openSpeedLoop.reachedNinetyPercent, ...
    'Zero gain must preserve the exact feedback-open force-trim limit.');

%% Deliberately broken case - reverse feedback removes thrust
% Reset both levers to baseline before changing only feedback sign.
broken=model(0.15,0.8,-1);
figure('Name','P15 broken reversed speed feedback');
subplot(1,2,1);
plot(baseline.time_s,baseline.trueAirspeed_mps,'LineWidth',1.7); hold on;
plot(broken.time_s,broken.trueAirspeed_mps,':','LineWidth',1.9);
plot(baseline.time_s,baseline.speedCommand_mps,'k--','LineWidth',1.2);
plot(broken.time_s,broken.stallSpeed_mps*ones(size(broken.time_s)), ...
    'r-.','LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('True airspeed (m/s)');
legend({'correct feedback','reversed feedback','command','stall boundary'}, ...
    'Location','best');
title('Wrong sign decelerates while the command asks for more speed');
subplot(1,2,2);
plot(baseline.time_s,100*baseline.throttleCommand,'LineWidth',1.7); hold on;
plot(broken.time_s,100*broken.throttleCommand,':','LineWidth',1.9);
grid on; xlabel('Time (s)'); ylabel('Commanded throttle (%)');
legend({'correct feedback','reversed feedback'},'Location','best');
title('The broken controller commands idle at step onset');
assert(broken.speedGain_per_s==baseline.speedGain_per_s && ...
    broken.throttleTimeConstant_s==baseline.throttleTimeConstant_s && ...
    broken.speedFeedbackSign==-1 && ...
    isequal(broken.time_s,baseline.time_s) && ...
    isequal(broken.speedCommand_mps,baseline.speedCommand_mps) && ...
    isequal(broken.trueAirspeed_mps(1:commandIndex), ...
    baseline.trueAirspeed_mps(1:commandIndex)) && ...
    isequal(broken.throttleActual(1:commandIndex), ...
    baseline.throttleActual(1:commandIndex)) && ...
    broken.thrustCommand_N(commandIndex)==0 && ...
    baseline.thrustCommand_N(commandIndex)>2500 && ...
    broken.finalSpeedError_mps>29 && ...
    broken.thrustCommandSaturationFraction>0.96 && ...
    broken.stallEnvelopeMaintained, ...
    'The broken case must isolate feedback sign and show sustained idle deceleration.');
fprintf(['Broken symptom: the +%.0f m/s error commands %.0f%% throttle; ' ...
    'after %.0f s speed is %.2f m/s and proper error is %.2f m/s, ' ...
    'still %.2f m/s above the stall boundary.\n'], ...
    broken.speedError_mps(commandIndex), ...
    100*broken.throttleCommand(commandIndex),broken.timeHorizon_s, ...
    broken.trueAirspeed_mps(end),broken.finalSpeedError_mps, ...
    broken.minimumStallMargin_mps);

%% Read and explain the broken assumption
% Reversing only feedback sign makes positive command-minus-speed error
% subtract thrust. The lower speed then creates more proper error, so the
% loop is positive feedback. Idle saturation prevents negative thrust but
% does not recover speed. The fixed trace remains above stall; it proves
% continued failure only through this observed horizon.
disp(['Broken mechanism: reversed speed feedback removes thrust, so drag ' ...
    'slows the aircraft and makes the proper command-minus-speed error grow.']);

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach-back: trace speed error through force request, throttle lag, ' ...
    'thrust-minus-drag acceleration, both lever tradeoffs, and the ' ...
    'recognizable reversed-feedback idle-throttle failure.']);
