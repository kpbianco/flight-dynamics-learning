%% P08 - Relate Stability Derivatives to Motion
% Guiding question:
% What inputs, observable effects, and failure modes matter when you relate Stability Derivatives to Motion?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P08 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - follow one derivative through the entire causal chain
% P07 prescribed separate roll, spiral, and Dutch-roll shapes. P08 stops
% prescribing those shapes and instead builds coupled motion from beta, p,
% r, and phi. Every aerodynamic rate first becomes p_hat=p*b/(2*V0) or
% r_hat=r*b/(2*V0), then a coefficient, dimensional force or moment, and
% finally a state acceleration.
disp('P07 identified modal signatures; P08 builds coupled motion from derivative contributions.');
disp(['Predict once: after a positive sideslip release, which initial signs should ' ...
    'stable dihedral C_l_beta<0 and weathercock C_n_beta>0 give p_dot and r_dot?']);

%% Baseline - release one deterministic positive sideslip perturbation
baseline=model(3,-0.50,0.18);
fprintf(['Baseline inputs: beta(0)=%+.1f deg, C_l_p=%+.2f, ' ...
    'C_n_beta=%+.2f /rad\n'],baseline.initialSideslip_deg, ...
    baseline.rollDampingDerivative_Cl_p, ...
    baseline.weathercockDerivative_Cn_beta_perRad);
fprintf(['Reference condition: V0=%.1f m/s, qbar=%.6f Pa, S=%.1f m^2, ' ...
    'b=%.1f m, m=%.0f kg, I_x=%.0f kg*m^2, I_z=%.0f kg*m^2\n'], ...
    baseline.referenceTrueAirspeed_mps, ...
    baseline.referenceDynamicPressure_Pa,baseline.wingArea_m2, ...
    baseline.wingSpan_m,baseline.mass_kg,baseline.rollInertia_kgm2, ...
    baseline.yawInertia_kgm2);
fprintf(['Rate normalization: b/(2*V0)=%.9f s. Initial Y=%+.3f N, ' ...
    'L=%+.3f N*m, N=%+.3f N*m.\n'], ...
    baseline.rateNormalizationTime_s,baseline.sideForce_N(1), ...
    baseline.rollMoment_Nm(1),baseline.yawMoment_Nm(1));
fprintf(['Initial tendencies: beta_dot=%+.6f deg/s, p_dot=%+.6f deg/s^2, ' ...
    'r_dot=%+.6f deg/s^2, phi_dot=%+.6f deg/s.\n'], ...
    baseline.sideslipRate_deg_s(1),baseline.rollAcceleration_deg_s2(1), ...
    baseline.yawAcceleration_deg_s2(1),baseline.bankRate_deg_s(1));
fprintf(['Observed metrics: first beta zero=%.2f s, peak |p|=%.5f deg/s ' ...
    'at %.2f s, peak |r|=%.5f deg/s at %.2f s, peak |phi|=%.5f deg.\n'], ...
    baseline.firstSideslipZeroCrossing_s,baseline.peakAbsRollRate_deg_s, ...
    baseline.peakRollRateTime_s,baseline.peakAbsYawRate_deg_s, ...
    baseline.peakYawRateTime_s,baseline.peakAbsBank_deg);
assert(baseline.isWithinSideslipLinearRange && ...
    baseline.isWithinBankLinearRange && ...
    baseline.isWithinBodyRateLearningRange, ...
    'The baseline must stay inside the declared local learning limits.');

%% Baseline state view - observe sideslip and yaw before roll and bank
figure('Name','P08 deterministic coupled-state baseline');
subplot(2,2,1);
plot(baseline.time_s,baseline.sideslip_deg,'LineWidth',1.6); hold on;
plot(baseline.time_s,zeros(size(baseline.time_s)),'k--');
grid on; xlabel('Time after sideslip release (s)');
ylabel('Sideslip beta (deg)'); title('Lateral displacement state');
subplot(2,2,2);
plot(baseline.time_s,baseline.yawRate_deg_s,'LineWidth',1.6); hold on;
plot(baseline.time_s,zeros(size(baseline.time_s)),'k--');
grid on; xlabel('Time after sideslip release (s)');
ylabel('Yaw rate r (deg/s)'); title('Weathercock-driven yaw response');
subplot(2,2,3);
plot(baseline.time_s,baseline.rollRate_deg_s,'LineWidth',1.6); hold on;
plot(baseline.time_s,zeros(size(baseline.time_s)),'k--');
grid on; xlabel('Time after sideslip release (s)');
ylabel('Roll rate p (deg/s)'); title('Dihedral-driven roll response');
subplot(2,2,4);
plot(baseline.time_s,baseline.bankAngle_deg,'LineWidth',1.6); hold on;
plot(baseline.time_s,zeros(size(baseline.time_s)),'k--');
grid on; xlabel('Time after sideslip release (s)');
ylabel('Bank angle phi (deg)'); title('Integrated coupled bank motion');

%% Baseline contribution view - a mode is not one derivative
figure('Name','P08 derivative contribution ledger');
subplot(1,2,1);
plot(baseline.time_s,[baseline.rollMomentCoefficientFromBeta; ...
    baseline.rollMomentCoefficientFromRollRate; ...
    baseline.rollMomentCoefficientFromYawRate], 'LineWidth',1.3);
grid on; xlabel('Time after sideslip release (s)');
ylabel('Roll-moment coefficient contribution (-)');
legend({'C_{l_beta} beta','C_{l_p} p-hat','C_{l_r} r-hat'}, ...
    'Location','best'); title('Roll-moment derivative ledger');
subplot(1,2,2);
plot(baseline.time_s,[baseline.yawMomentCoefficientFromBeta; ...
    baseline.yawMomentCoefficientFromRollRate; ...
    baseline.yawMomentCoefficientFromYawRate], 'LineWidth',1.3);
grid on; xlabel('Time after sideslip release (s)');
ylabel('Yaw-moment coefficient contribution (-)');
legend({'C_{n_beta} beta','C_{n_p} p-hat','C_{n_r} r-hat'}, ...
    'Location','best'); title('Yaw-moment derivative ledger');

%% Lever 1 - sweep C_l_p with every other input and derivative reset
rollDampingSweep_Cl_p=[-0.30 -0.40 -0.50 -0.65 -0.80];
rollRateSweep_deg_s=zeros(numel(rollDampingSweep_Cl_p),baseline.sampleCount);
rollPeakSweep_deg_s=zeros(size(rollDampingSweep_Cl_p));
bankPeakSweep_deg=zeros(size(rollDampingSweep_Cl_p));
initialRollAccelerationSweep_deg_s2=zeros(size(rollDampingSweep_Cl_p));
for k=1:numel(rollDampingSweep_Cl_p)
    sample=model(3,rollDampingSweep_Cl_p(k),0.18);
    rollRateSweep_deg_s(k,:)=sample.rollRate_deg_s;
    rollPeakSweep_deg_s(k)=sample.peakAbsRollRate_deg_s;
    bankPeakSweep_deg(k)=sample.peakAbsBank_deg;
    initialRollAccelerationSweep_deg_s2(k)= ...
        sample.rollAcceleration_deg_s2(1);
    matrixDifference=sample.stateMatrix-baseline.stateMatrix;
    matrixDifference(2,2)=0;
    assert(all(matrixDifference(:)==0), ...
        'The C_l_p sweep changed a state-matrix entry other than A(2,2).');
    assert(sample.initialSideslip_deg==baseline.initialSideslip_deg && ...
        sample.weathercockDerivative_Cn_beta_perRad== ...
        baseline.weathercockDerivative_Cn_beta_perRad, ...
        'The C_l_p sweep failed to hold the release and C_n_beta fixed.');
end

%% Changed view - stronger roll damping reduces rate and bank excursions
figure('Name','P08 roll-damping derivative sweep');
subplot(1,2,1);
plot(baseline.time_s,rollRateSweep_deg_s,'LineWidth',1.2);
grid on; xlabel('Time after sideslip release (s)');
ylabel('Roll rate p (deg/s)');
legend(compose('C_l_p = %.2f',rollDampingSweep_Cl_p),'Location','best');
title('Only the roll-rate derivative changes');
subplot(1,2,2);
plot(rollDampingSweep_Cl_p,rollPeakSweep_deg_s,'o-','LineWidth',1.4); hold on;
plot(rollDampingSweep_Cl_p,bankPeakSweep_deg,'s-','LineWidth',1.4);
grid on; xlabel('Roll damping derivative C_l_p (-)');
ylabel('Peak magnitude (deg/s or deg)');
legend({'peak |p| (deg/s)','peak |phi| (deg)'},'Location','best');
title('More-negative C_l_p opposes developed roll rate');
assert(all(diff(rollPeakSweep_deg_s)<0) && ...
    all(diff(bankPeakSweep_deg)<0) && ...
    max(abs(initialRollAccelerationSweep_deg_s2- ...
    initialRollAccelerationSweep_deg_s2(1)))<1e-12, ...
    'The roll-damping sweep must reduce peaks without changing initial p-dot.');

%% Read and explain - mechanism for lever 1
disp(['Mechanism: C_l_p multiplies p-hat=p*b/(2*V0). It contributes no moment ' ...
    'at release because p(0)=0; after roll rate develops, a more-negative value ' ...
    'opposes p more strongly and reduces both peak rate and integrated bank.']);

%% Lever 2 - reset C_l_p, then sweep weathercock stability C_n_beta
weathercockSweep_Cn_beta_perRad=[0 0.06 0.12 0.18 0.24];
yawRateSweep_deg_s=zeros(numel(weathercockSweep_Cn_beta_perRad), ...
    baseline.sampleCount);
initialYawAccelerationSweep_deg_s2=zeros( ...
    size(weathercockSweep_Cn_beta_perRad));
firstZeroSweep_s=zeros(size(weathercockSweep_Cn_beta_perRad));
yawPeakSweep_deg_s=zeros(size(weathercockSweep_Cn_beta_perRad));
for k=1:numel(weathercockSweep_Cn_beta_perRad)
    sample=model(3,-0.50,weathercockSweep_Cn_beta_perRad(k));
    yawRateSweep_deg_s(k,:)=sample.yawRate_deg_s;
    initialYawAccelerationSweep_deg_s2(k)= ...
        sample.yawAcceleration_deg_s2(1);
    firstZeroSweep_s(k)=sample.firstSideslipZeroCrossing_s;
    yawPeakSweep_deg_s(k)=sample.peakAbsYawRate_deg_s;
    matrixDifference=sample.stateMatrix-baseline.stateMatrix;
    matrixDifference(3,1)=0;
    assert(all(matrixDifference(:)==0), ...
        'The C_n_beta sweep changed a state-matrix entry other than A(3,1).');
    assert(sample.initialSideslip_deg==baseline.initialSideslip_deg && ...
        sample.rollDampingDerivative_Cl_p== ...
        baseline.rollDampingDerivative_Cl_p, ...
        'The C_n_beta sweep failed to reset the release and C_l_p.');
end

%% Changed view - weathercock stability changes initial yaw and coupled timing
figure('Name','P08 weathercock derivative sweep');
subplot(1,3,1);
plot(baseline.time_s,yawRateSweep_deg_s,'LineWidth',1.2);
grid on; xlabel('Time after sideslip release (s)');
ylabel('Yaw rate r (deg/s)');
legend(compose('C_n_beta = %.2f /rad', ...
    weathercockSweep_Cn_beta_perRad),'Location','best');
title('Coupled yaw response');
subplot(1,3,2);
plot(weathercockSweep_Cn_beta_perRad, ...
    initialYawAccelerationSweep_deg_s2,'o-','LineWidth',1.4);
grid on; xlabel('Weathercock derivative C_n_beta (1/rad)');
ylabel('Initial yaw acceleration (deg/s^2)');
title('C_n_beta maps directly into N_beta');
subplot(1,3,3);
plot(weathercockSweep_Cn_beta_perRad,firstZeroSweep_s,'s-','LineWidth',1.4);
grid on; xlabel('Weathercock derivative C_n_beta (1/rad)');
ylabel('First beta zero crossing (s)');
title('More restoring yaw advances the crossing');
assert(initialYawAccelerationSweep_deg_s2(1)==0 && ...
    all(diff(initialYawAccelerationSweep_deg_s2)>0) && ...
    all(diff(firstZeroSweep_s)<0) && all(diff(yawPeakSweep_deg_s)>0), ...
    'The weathercock sweep must expose its direct and coupled effects.');

%% Read and explain - mechanism for lever 2
disp(['Mechanism: positive C_n_beta turns positive beta into a nose-right yaw ' ...
    'acceleration. Through beta_dot=Y/(mV)-r+g*phi/V, larger yaw response drives ' ...
    'beta through zero sooner. At C_n_beta=0, C_n_p*p still creates later yaw: ' ...
    'one derivative is not one mode.']);

%% Broken case - omit b/(2*V0) from the C_l_p rate derivative
% The wrong trace is smooth and stays small, which makes it misleading.
% C_l_p is defined against p_hat, so using dimensional p directly creates a
% unit-inconsistent A(2,2) with numerically excessive roll damping.
figure('Name','P08 broken roll-rate normalization');
subplot(1,2,1);
plot(baseline.time_s,baseline.rollRate_deg_s,'LineWidth',1.6); hold on;
plot(baseline.time_s,baseline.brokenRollRate_deg_s,'--','LineWidth',1.6);
grid on; xlabel('Time after sideslip release (s)');
ylabel('Roll rate p (deg/s)');
legend({'correct p-hat scaling','broken dimensional-p use'},'Location','best');
title('A smooth trace can still have the wrong units');
subplot(1,2,2);
plot(baseline.time_s,baseline.bankAngle_deg,'LineWidth',1.6); hold on;
plot(baseline.time_s,baseline.brokenBankAngle_deg,'--','LineWidth',1.6);
grid on; xlabel('Time after sideslip release (s)');
ylabel('Bank angle phi (deg)');
legend({'correct p-hat scaling','broken dimensional-p use'},'Location','best');
title('Excess numeric damping hides coupled roll motion');
fprintf(['Broken normalization: correct A(2,2)=%+.5f 1/s; the broken ' ...
    'numeric entry is %+.5f with incompatible 1/s^2 units. Their SI ' ...
    'numeric-value quotient is %.5f 1/s, not a dimensionless multiplier; ' ...
    'peak |p| changes %.5f -> %.5f deg/s and peak |phi| changes ' ...
    '%.5f -> %.5f deg.\n'],baseline.stateMatrix(2,2), ...
    baseline.brokenStateMatrix(2,2),baseline.brokenA22NumericRatio_per_s, ...
    baseline.peakAbsRollRate_deg_s, ...
    baseline.brokenPeakAbsRollRate_deg_s,baseline.peakAbsBank_deg, ...
    baseline.brokenPeakAbsBank_deg);
assert(abs(baseline.brokenA22NumericRatio_per_s- ...
    2*baseline.referenceTrueAirspeed_mps/baseline.wingSpan_m)<1e-12 && ...
    baseline.peakAbsRollRate_deg_s> ...
    5*baseline.brokenPeakAbsRollRate_deg_s && ...
    baseline.peakAbsBank_deg>8*baseline.brokenPeakAbsBank_deg, ...
    'The omitted rate normalization must produce the recognizable symptom.');

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach back in two sentences: trace beta, p-hat, and r-hat through ' ...
    'coefficients, dimensional loads, accelerations, and coupled states; then ' ...
    'explain why omitting b/(2*V0) can look smooth while being wrong.']);
