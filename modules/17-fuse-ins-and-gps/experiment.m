%% P17 - Fuse INS and GPS
% Guiding question:
% What inputs, observable effects, and failure modes matter when you fuse INS and GPS?
%
% Predict once: with a small constant INS acceleration bias, will position
% error grow linearly or quadratically before GPS corrections are applied?

%% Baseline - compare truth, dead reckoning, GPS, and fused navigation
baseline=model(0.04,1,1);
fprintf(['Baseline: INS-only final error %.3f m, fused RMS %.3f m, ' ...
    '%d accepted GPS fixes, %d rejected outlier.\n'], ...
    baseline.insOnlyFinalPositionError_m,baseline.fusedPositionRMS_m, ...
    baseline.gpsAcceptedCount,baseline.gpsRejectedCount);

figure('Name','P17 Baseline Navigation');
subplot(2,1,1);
plot(baseline.time_s,baseline.northPositionTruth_m,'k-', ...
    'LineWidth',1.8);
hold on;
plot(baseline.time_s,baseline.northPositionINSOnly_m,'--', ...
    'LineWidth',1.4);
plot(baseline.time_s,baseline.northPositionFused_m,'LineWidth',1.6);
plot(baseline.time_s(baseline.gpsUpdateMask), ...
    baseline.gpsPositionMeasurement_m(baseline.gpsUpdateMask),'o', ...
    'MarkerSize',3);
plot(baseline.outlierTime_s, ...
    baseline.gpsPositionMeasurement_m(baseline.outlierIndex),'rx', ...
    'MarkerSize',10,'LineWidth',2);
grid on;
xlabel('Time (s)'); ylabel('North position (m)');
legend({'truth','INS only','gated fused','GPS fixes','GPS outlier'}, ...
    'Location','best');
title('High-rate prediction anchored by absolute fixes');

subplot(2,1,2);
plot(baseline.time_s,baseline.northPositionINSOnlyError_m,'--', ...
    'LineWidth',1.4);
hold on;
plot(baseline.time_s,baseline.northPositionFusedError_m, ...
    'LineWidth',1.6);
plot(baseline.time_s,baseline.expectedINSOnlyPositionError_m,':', ...
    'LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('North position error (m)');
legend({'INS only','gated fused','0.5 b_a t^2'},'Location','best');
title('Bias accumulates; accepted GPS innovations bound the fused error');

%% Processing view - inspect prediction, innovation, gate, and correction
figure('Name','P17 Prediction And Correction');
subplot(3,1,1);
plot(baseline.time_s,baseline.northPositionPredicted_m- ...
    baseline.northPositionTruth_m,'Color',[0.65 0.65 0.65], ...
    'LineWidth',1.1);
hold on;
plot(baseline.time_s,baseline.northPositionFusedError_m, ...
    'LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('Position error (m)');
legend({'before GPS correction','after correction'},'Location','best');
title('Prediction comes before correction at each fix');

subplot(3,1,2);
stem(baseline.time_s(baseline.gpsUpdateMask), ...
    baseline.gpsInnovation_m(baseline.gpsUpdateMask),'filled', ...
    'MarkerSize',3);
hold on;
plot(baseline.time_s,baseline.innovationGate_m*ones(size(baseline.time_s)), ...
    'r--');
plot(baseline.time_s,-baseline.innovationGate_m*ones(size(baseline.time_s)), ...
    'r--');
grid on;
xlabel('Time (s)'); ylabel('GPS innovation (m)');
title('|innovation| <= 25 m is accepted');

subplot(3,1,3);
stem(baseline.time_s(baseline.gpsUpdateMask), ...
    baseline.gpsPositionCorrection_m(baseline.gpsUpdateMask),'filled', ...
    'MarkerSize',3);
grid on;
xlabel('Time (s)'); ylabel('Position correction (m)');
title('The rejected outlier produces exactly zero correction');

%% Lever 1 - sweep INS acceleration bias while GPS error stays reset
insBiasSweep_mps2=[0 0.02 0.04 0.06 0.08];
biasFusedRMS_m=zeros(size(insBiasSweep_mps2));
biasINSFinalError_m=zeros(size(insBiasSweep_mps2));
biasFusedPeakError_m=zeros(size(insBiasSweep_mps2));

figure('Name','P17 INS Bias Sweep');
subplot(2,1,1);
hold on;
for k=1:numel(insBiasSweep_mps2)
    sample=model(insBiasSweep_mps2(k),baseline.gpsPositionErrorRms_m,1);
    plot(sample.time_s,sample.northPositionFusedError_m,'LineWidth',1.2);
    biasFusedRMS_m(k)=sample.fusedPositionRMS_m;
    biasINSFinalError_m(k)=sample.insOnlyFinalPositionError_m;
    biasFusedPeakError_m(k)=sample.fusedPeakAbsolutePositionError_m;
end
grid on;
xlabel('Time (s)'); ylabel('Fused North position error (m)');
legend(compose('b_a = %.2f m/s^2',insBiasSweep_mps2), ...
    'Location','best');
title('Lever 1: more residual acceleration bias loads each prediction');

subplot(2,1,2);
plot(insBiasSweep_mps2,biasINSFinalError_m,'o-','LineWidth',1.5);
hold on;
plot(insBiasSweep_mps2,biasFusedPeakError_m,'s-','LineWidth',1.5);
grid on;
xlabel('INS acceleration bias (m/s^2)'); ylabel('Position error (m)');
legend({'INS-only final','fused peak absolute'},'Location','best');
title('Mechanism: integration turns constant bias into quadratic drift');

%% Lever 2 - reset bias, then sweep deterministic GPS position-error RMS
gpsPositionErrorSweep_m=[0 0.5 1 2 4];
gpsMeasuredRMS_m=zeros(size(gpsPositionErrorSweep_m));
gpsFusedRMS_m=zeros(size(gpsPositionErrorSweep_m));
gpsFusedPeakError_m=zeros(size(gpsPositionErrorSweep_m));

figure('Name','P17 GPS Error Sweep');
subplot(2,1,1);
hold on;
for k=1:numel(gpsPositionErrorSweep_m)
    sample=model(baseline.insAccelerationBias_mps2, ...
        gpsPositionErrorSweep_m(k),1);
    plot(sample.time_s,sample.northPositionFusedError_m,'LineWidth',1.2);
    gpsMeasuredRMS_m(k)=sample.gpsPositionErrorRmsMeasured_m;
    gpsFusedRMS_m(k)=sample.fusedPositionRMS_m;
    gpsFusedPeakError_m(k)=sample.fusedPeakAbsolutePositionError_m;
end
grid on;
xlabel('Time (s)'); ylabel('Fused North position error (m)');
legend(compose('GPS RMS = %.1f m',gpsPositionErrorSweep_m), ...
    'Location','best');
title('Lever 2: accepted GPS error appears through discrete corrections');

subplot(2,1,2);
plot(gpsPositionErrorSweep_m,gpsMeasuredRMS_m,'o-', ...
    'LineWidth',1.5);
hold on;
plot(gpsPositionErrorSweep_m,gpsFusedRMS_m,'s-', ...
    'LineWidth',1.5);
grid on;
xlabel('Selected GPS position-error RMS (m)'); ylabel('Measured RMS (m)');
legend({'nominal GPS error','fused position error'},'Location','best');
title('Mechanism: absolute fixes bound drift but also inject their error');

%% Limits and deliberately broken case - disable only innovation gating
ideal=model(0,0,1);
insOnly=model(baseline.insAccelerationBias_mps2, ...
    baseline.gpsPositionErrorRms_m,0);
broken=model(baseline.insAccelerationBias_mps2, ...
    baseline.gpsPositionErrorRms_m,-1);
fprintf(['Ideal max fused error %.3g m; INS-only final error %.3f m. ' ...
    'At the outlier, correct/broken corrections are %.3f/%.3f m.\n'], ...
    ideal.fusedPeakAbsolutePositionError_m, ...
    insOnly.fusedFinalPositionError_m, ...
    baseline.outlierPositionCorrection_m, ...
    broken.outlierPositionCorrection_m);

outlierWindow=baseline.time_s>=27 & baseline.time_s<=35;
figure('Name','P17 Broken GPS Gate');
subplot(2,1,1);
plot(baseline.time_s(outlierWindow), ...
    baseline.northPositionFusedError_m(outlierWindow),'LineWidth',1.7);
hold on;
plot(broken.time_s(outlierWindow), ...
    broken.northPositionFusedError_m(outlierWindow),'--','LineWidth',1.7);
plot(baseline.outlierTime_s, ...
    broken.northPositionFusedError_m(broken.outlierIndex),'rx', ...
    'MarkerSize',10,'LineWidth',2);
grid on;
xlabel('Time (s)'); ylabel('North position error (m)');
legend({'gated correct','BROKEN accept-all','accepted outlier sample'}, ...
    'Location','best');
title('Same sensor stream; only the gate decision changes');

subplot(2,1,2);
stem(baseline.time_s(baseline.gpsUpdateMask & outlierWindow), ...
    baseline.gpsInnovation_m(baseline.gpsUpdateMask & outlierWindow), ...
    'filled','MarkerSize',3);
hold on;
stem(broken.time_s(broken.gpsUpdateMask & outlierWindow), ...
    broken.gpsPositionCorrection_m(broken.gpsUpdateMask & outlierWindow), ...
    'MarkerSize',3);
plot(baseline.time_s(outlierWindow), ...
    baseline.innovationGate_m*ones(1,sum(outlierWindow)),'r--');
plot(baseline.time_s(outlierWindow), ...
    -baseline.innovationGate_m*ones(1,sum(outlierWindow)),'r--');
grid on;
xlabel('Time (s)'); ylabel('Innovation or correction (m)');
legend({'innovation','BROKEN position correction','gate'}, ...
    'Location','best');
title('Accept-all violates the bounded-innovation assumption');

%% Explain, check, and teach back
% Reset to model(0.04,1,1). The INS supplies continuity between fixes; GPS
% supplies an absolute residual. Bias integration makes dead reckoning drift,
% nominal GPS error enters accepted corrections, and an innovation gate keeps
% one implausible fix from becoming a state jump. This deterministic model is
% not a covariance filter, receiver, attitude mechanization, or flight test.
clear run_checks;
run_checks;
