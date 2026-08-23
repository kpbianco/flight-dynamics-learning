%% P05 - See Longitudinal Static Stability
% Guiding question:
% What inputs, observable effects, and failure modes matter when you see Longitudinal Static Stability?
% Replace only figures owned by this learning harness; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P[0-9][0-9] ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - add pitching moment to the P04 force-trim reference
% P04 supplied q=1325.01 Pa and alpha=3.42 deg for its level force balance.
% P05 declares that state locally retrimmed, then asks how a small delta
% alpha changes nose-up-positive C_m. Predict once: after a positive delta
% alpha, should a statically stable aircraft create nose-up or nose-down moment?
disp('Static stability is the initial moment tendency after a small disturbance, not the later motion.');
disp('With nose-up C_m positive, a stable stick-fixed slope dC_m/dalpha is negative.');

%% Baseline - one deterministic restoring response
baseline=model(30,20,2,0);
fprintf(['Baseline: cg=%.1f%% MAC; S_t/S=%.1f%%; delta alpha=%+.1f deg; ' ...
    'delta elevator=%+.1f deg\n'],baseline.cgPosition_percentMAC, ...
    baseline.tailAreaRatio_percent,baseline.angleOfAttackPerturbation_deg, ...
    baseline.elevatorPerturbation_deg);
fprintf(['P04 reference: q=%.2f Pa; alpha_trim=%.3f deg; ' ...
    'neutral point=%.3f%% MAC; static margin=%.3f%% MAC\n'], ...
    baseline.referenceDynamicPressure_Pa, ...
    baseline.referenceTrimAngleOfAttack_deg,baseline.neutralPoint_percentMAC, ...
    baseline.staticMargin_percentMAC);
fprintf(['C_m_alpha=%.4f /rad; delta C_m=%+.5f; delta M=%+.2f N*m; %s\n'], ...
    baseline.pitchingMomentSlope_perRad,baseline.pitchingMomentCoefficient, ...
    baseline.pitchingMoment_Nm,baseline.stabilityLabel);

alphaGrid_deg=-5:0.5:5;
momentCoefficientGrid=zeros(size(alphaGrid_deg));
momentGrid_Nm=zeros(size(alphaGrid_deg));
for k=1:numel(alphaGrid_deg)
    sample=model(30,20,alphaGrid_deg(k),0);
    momentCoefficientGrid(k)=sample.pitchingMomentCoefficient;
    momentGrid_Nm(k)=sample.pitchingMoment_Nm;
end

figure('Name','P05 deterministic baseline');
subplot(1,2,1);
plot(alphaGrid_deg,momentCoefficientGrid,'LineWidth',1.6); hold on;
plot(alphaGrid_deg,zeros(size(alphaGrid_deg)),'k--','LineWidth',1.0);
plot(baseline.angleOfAttackPerturbation_deg, ...
    baseline.pitchingMomentCoefficient,'o','MarkerSize',9,'LineWidth',2);
grid on; xlabel('Angle-of-attack perturbation, delta alpha (deg)');
ylabel('Pitching-moment increment, delta C_m (-)');
legend({'baseline slope','zero moment','selected disturbance'},'Location','best');
title('Negative slope gives restoring moment');
subplot(1,2,2);
bar([baseline.wingMomentSlope_perRad baseline.tailMomentSlope_perRad ...
    baseline.pitchingMomentSlope_perRad]); grid on;
set(gca,'XTickLabel',{'wing','tail','total'});
ylabel('dC_m/dalpha (1/rad)');
title('Visible component derivative buildup');

%% Lever 1 - sweep CG position with tail size and disturbance fixed
cgSweep_percentMAC=[20 28 36 44 50.8468335787924 56];
cgNeutralPoint_percentMAC=zeros(size(cgSweep_percentMAC));
cgStaticMargin_percentMAC=zeros(size(cgSweep_percentMAC));
cgMomentSlope_perRad=zeros(size(cgSweep_percentMAC));
cgPitchingMoment_Nm=zeros(size(cgSweep_percentMAC));
cgStable=false(size(cgSweep_percentMAC));
for k=1:numel(cgSweep_percentMAC)
    sample=model(cgSweep_percentMAC(k),20,2,0);
    cgNeutralPoint_percentMAC(k)=sample.neutralPoint_percentMAC;
    cgStaticMargin_percentMAC(k)=sample.staticMargin_percentMAC;
    cgMomentSlope_perRad(k)=sample.pitchingMomentSlope_perRad;
    cgPitchingMoment_Nm(k)=sample.pitchingMoment_Nm;
    cgStable(k)=sample.isStaticallyStable;
end

%% Changed view - moving CG aft removes restoring leverage
figure('Name','P05 CG-position sweep');
subplot(1,2,1);
plot(cgSweep_percentMAC,cgStaticMargin_percentMAC,'o-','LineWidth',1.5); hold on;
plot(cgSweep_percentMAC,zeros(size(cgSweep_percentMAC)),'k--');
grid on; xlabel('CG position aft of MAC leading edge (% MAC)');
ylabel('Static margin h_n - h_c_g (% MAC)');
title('CG crossing the neutral point changes the sign');
subplot(1,2,2);
plot(cgSweep_percentMAC,cgPitchingMoment_Nm,'s-','LineWidth',1.5); hold on;
plot(cgSweep_percentMAC,zeros(size(cgSweep_percentMAC)),'k--');
grid on; xlabel('CG position aft of MAC leading edge (% MAC)');
ylabel('Moment after +2 deg delta alpha (N*m)');
title('Nose-down restoring moment becomes nose-up');
fprintf('CG sweep: %d of %d points are stable; h_n stays %.3f%% MAC.\n', ...
    sum(cgStable),numel(cgStable),cgNeutralPoint_percentMAC(1));

%% Read and explain - mechanism for lever 1
disp(['Mechanism: moving the CG aft shortens the positive static margin. ' ...
    'At h_c_g=h_n the first-order alpha moment vanishes; aft of h_n it reinforces the disturbance.']);

%% Lever 2 - reset CG, then sweep horizontal-tail area independently
tailAreaSweep_percent=[0 3.61689814814815 5 10 15 20 25];
tailNeutralPoint_percentMAC=zeros(size(tailAreaSweep_percent));
tailStaticMargin_percentMAC=zeros(size(tailAreaSweep_percent));
tailMomentSlope_perRad=zeros(size(tailAreaSweep_percent));
tailPitchingMoment_Nm=zeros(size(tailAreaSweep_percent));
tailStable=false(size(tailAreaSweep_percent));
for k=1:numel(tailAreaSweep_percent)
    sample=model(30,tailAreaSweep_percent(k),2,0);
    tailNeutralPoint_percentMAC(k)=sample.neutralPoint_percentMAC;
    tailStaticMargin_percentMAC(k)=sample.staticMargin_percentMAC;
    tailMomentSlope_perRad(k)=sample.pitchingMomentSlope_perRad;
    tailPitchingMoment_Nm(k)=sample.pitchingMoment_Nm;
    tailStable(k)=sample.isStaticallyStable;
end

%% Changed view - more tail area moves the neutral point aft
figure('Name','P05 horizontal-tail-area sweep');
subplot(1,2,1);
plot(tailAreaSweep_percent,tailNeutralPoint_percentMAC,'o-','LineWidth',1.5); hold on;
plot(tailAreaSweep_percent,30*ones(size(tailAreaSweep_percent)),'k--');
grid on; xlabel('Horizontal-tail area ratio S_t/S (%)');
ylabel('Station aft of MAC leading edge (% MAC)');
legend({'neutral point','fixed CG'},'Location','best');
title('Tail lift contribution moves h_n aft');
subplot(1,2,2);
plot(tailAreaSweep_percent,tailPitchingMoment_Nm,'s-','LineWidth',1.5); hold on;
plot(tailAreaSweep_percent,zeros(size(tailAreaSweep_percent)),'k--');
grid on; xlabel('Horizontal-tail area ratio S_t/S (%)');
ylabel('Moment after +2 deg delta alpha (N*m)');
title('Tail size changes the restoring response');
fprintf('Tail-area sweep: %d of %d points are stable at cg=30%% MAC.\n', ...
    sum(tailStable),numel(tailStable));

%% Read and explain - mechanism for lever 2
disp(['Mechanism: a larger tail adds lift-curve slope far aft of the CG. ' ...
    'That shifts the neutral point aft and makes C_m_alpha more negative at the reset CG.']);

%% Broken case - reverse the static-margin sign convention
% Correct static margin is h_n-h_c_g. The broken calculation uses the
% opposite sign while still calling positive margin stable, so every alpha
% response reverses and a positive disturbance produces reinforcing nose-up moment.
brokenStaticMargin_fractionMAC=(baseline.cgPosition_percentMAC- ...
    baseline.neutralPoint_percentMAC)/100;
brokenMomentSlope_perRad=-baseline.aircraftLiftCurveSlope_perRad* ...
    brokenStaticMargin_fractionMAC;
brokenMomentCoefficientGrid=brokenMomentSlope_perRad*(alphaGrid_deg*pi/180);
brokenPitchingMoment_Nm=baseline.momentScale_Nm*brokenMomentSlope_perRad* ...
    baseline.angleOfAttackPerturbation_rad;

figure('Name','P05 broken static-margin sign');
subplot(1,2,1);
plot(alphaGrid_deg,momentCoefficientGrid,'LineWidth',1.6); hold on;
plot(alphaGrid_deg,brokenMomentCoefficientGrid,'--','LineWidth',1.6);
plot(alphaGrid_deg,zeros(size(alphaGrid_deg)),'k:');
grid on; xlabel('Angle-of-attack perturbation, delta alpha (deg)');
ylabel('Pitching-moment increment, delta C_m (-)');
legend({'correct restoring slope','broken reversed slope','zero moment'}, ...
    'Location','best');
title('One reversed distance sign changes the tendency');
subplot(1,2,2);
bar([baseline.alphaMoment_Nm brokenPitchingMoment_Nm]); grid on;
set(gca,'XTickLabel',{'correct','broken'});
ylabel('Moment after +2 deg delta alpha (N*m)');
title('Broken sign reinforces the disturbance');
fprintf(['Broken symptom: C_m_alpha changes from %.4f to %.4f /rad and ' ...
    'the +2 deg moment changes from %+.2f to %+.2f N*m.\n'], ...
    baseline.pitchingMomentSlope_perRad,brokenMomentSlope_perRad, ...
    baseline.alphaMoment_Nm,brokenPitchingMoment_Nm);
assert(baseline.alphaMoment_Nm<0 && brokenPitchingMoment_Nm>0, ...
    'The broken sign must turn the restoring baseline moment into a reinforcing moment.');

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach back in two sentences: connect CG and tail area to h_n, static margin, and C_m_alpha; ' ...
    'then diagnose the sign error from the direction of the first moment after positive delta alpha.']);
