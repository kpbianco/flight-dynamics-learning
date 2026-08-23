%% P12 - Validate Energy and Frame Conventions
% Guiding question:
% What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P12 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - one trajectory, two coordinate descriptions, one work-energy audit
% P11 made body and North-East-Down (NED) signs explicit for specific-force
% measurements. P12 uses the same ideal quantity as a transparent input:
%
%   a_n       = C_body_to_ned f_b + g_n
%   power     = m f_b dot v_b = m f_n dot v_n
%   E(t)-E(0) = integral(power dt)
%
% A proper DCM preserves dot products. NED Down is positive, but altitude
% is h=-Down, so U=m g h=-m g Down. The fixed attitude and analytic motion
% isolate these bookkeeping rules; they are not an aircraft propagator.
disp('P12 audits one analytic work-energy trajectory in body and NED frames.');
disp(['Predict once: during a climb, does positive NED Down increase or ' ...
    'decrease, and which sign must potential energy use?']);

%% Baseline - fixed pitch with forward non-gravity specific force
baseline=model(1.5,30);
fprintf(['Baseline inputs: forward specific force %.2f m/s^2, heading %.1f deg, ' ...
    'fixed pitch %.1f deg, speed %.1f m/s, altitude %.1f m.\n'], ...
    baseline.forwardSpecificForce_mps2,baseline.headingAngle_deg, ...
    baseline.pitchAngle_deg,baseline.initialSpeed_mps, ...
    baseline.initialAltitude_m);
fprintf(['Initial NED velocity: North %.3f, East %.3f, Down %.3f m/s; ' ...
    'apex gain %.3f m at %.3f s.\n'], ...
    baseline.initialVelocityNED_mps(1), ...
    baseline.initialVelocityNED_mps(2), ...
    baseline.initialVelocityNED_mps(3), ...
    baseline.apexAltitudeGain_m,baseline.apexTime_s);
fprintf(['Final non-gravity work %.3f MJ; maximum correct balance residual ' ...
    '%.3g J; broken peak drift %.3f MJ.\n'], ...
    baseline.finalWorkInput_J/1e6,baseline.maxEnergyBalanceResidual_J, ...
    baseline.peakBrokenEnergyDrift_J/1e6);
assert(baseline.sampleCount==301 && baseline.intervalCount==300 && ...
    baseline.maxFrameRoundTripError_mps<1e-12 && ...
    baseline.maxKineticFrameDifference_J<1e-7 && ...
    baseline.maxPowerFrameDifference_W<1e-7 && ...
    baseline.maxEnergyBalanceResidual_J<1e-7 && ...
    baseline.dcmOrthonormalityError<1e-12 && ...
    baseline.dcmDeterminantError<1e-12, ...
    'The baseline must close its declared frame, power, and energy invariants.');

%% Baseline view 1 - the same motion in NED geometry and frame components
figure('Name','P12 baseline trajectory and frame components');
subplot(1,2,1);
plot3(baseline.positionNED_m(1,:),baseline.positionNED_m(2,:), ...
    baseline.altitude_m,'LineWidth',1.7);
grid on; xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude, -Down (m)');
title('Analytic NED trajectory');
subplot(1,2,2);
plot(baseline.time_s,baseline.velocityNED_mps.','LineWidth',1.4);
grid on; xlabel('Time (s)'); ylabel('NED velocity component (m/s)');
legend({'North','East','Down'},'Location','best');
title('Nose-up starts with negative Down speed');

%% Baseline view 2 - energy change closes to non-gravity work
figure('Name','P12 baseline work and energy ledger');
subplot(1,3,1);
plot(baseline.time_s, ...
    (baseline.mechanicalEnergy_J-baseline.mechanicalEnergy_J(1))/1e3, ...
    'LineWidth',1.7); hold on;
plot(baseline.time_s,baseline.workInput_J/1e3,'k--','LineWidth',1.4);
grid on; xlabel('Time (s)'); ylabel('Energy or work (kJ)');
legend({'mechanical energy change','non-gravity work'},'Location','best');
title('Work explains changing mechanical energy');
subplot(1,3,2);
plot(baseline.time_s,baseline.energyBalanceResidual_J,'LineWidth',1.7); hold on;
plot(baseline.time_s,baseline.kineticFrameDifference_J,'--','LineWidth',1.4);
grid on; xlabel('Time (s)'); ylabel('Energy residual (J)');
legend({'energy minus work','body T minus NED T'},'Location','best');
title('Energy and velocity norm close');
subplot(1,3,3);
plot(baseline.time_s,baseline.powerFrameDifference_W,':','LineWidth',1.7);
grid on; xlabel('Time (s)'); ylabel('Power residual (W)');
title('Body and NED dot products close');

%% Lever 1 - hold heading fixed and sweep only forward specific force
specificForceSweep_mps2=[0 0.75 1.5 2.25 3.0];
forceAltitude_m=zeros(numel(specificForceSweep_mps2),baseline.sampleCount);
forceWork_J=zeros(size(specificForceSweep_mps2));
forceApexGain_m=zeros(size(specificForceSweep_mps2));
forceFinalRange_m=zeros(size(specificForceSweep_mps2));
for k=1:numel(specificForceSweep_mps2)
    sample=model(specificForceSweep_mps2(k),30);
    forceAltitude_m(k,:)=sample.altitude_m;
    forceWork_J(k)=sample.finalWorkInput_J;
    forceApexGain_m(k)=sample.apexAltitudeGain_m;
    forceFinalRange_m(k)=sample.horizontalRange_m;
    assert(sample.headingAngle_deg==baseline.headingAngle_deg && ...
        isequal(sample.bodyToNED,baseline.bodyToNED) && ...
        isequal(sample.initialVelocityNED_mps, ...
        baseline.initialVelocityNED_mps) && ...
        sample.maxEnergyBalanceResidual_J<1e-7, ...
        ['The specific-force sweep must preserve heading, DCM, and initial ' ...
        'state while energy change follows work.']);
end

%% Changed view - specific force changes work and the zoom-climb arc
figure('Name','P12 specific-force sweep');
subplot(1,3,1);
plot(baseline.time_s,forceAltitude_m,'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Altitude, -Down (m)');
legend(compose('f_x %.2f m/s^2',specificForceSweep_mps2), ...
    'Location','best');
title('Forward force has an upward NED component');
subplot(1,3,2);
plot(specificForceSweep_mps2,forceWork_J/1e3,'o-','LineWidth',1.5);
grid on; xlabel('Forward specific force (m/s^2)');
ylabel('Final non-gravity work (kJ)');
title('More force supplies more work');
subplot(1,3,3);
plot(specificForceSweep_mps2,forceApexGain_m,'s-','LineWidth',1.5);
grid on; xlabel('Forward specific force (m/s^2)');
ylabel('Apex altitude gain (m)');
title('Upward force component raises the apex');
assert(forceWork_J(1)==0 && all(diff(forceWork_J)>0) && ...
    all(diff(forceApexGain_m)>0) && all(diff(forceFinalRange_m)>0), ...
    'Increasing forward specific force must increase work, apex, and range.');

%% Read and explain lever 1
% At the zero-force limit, a free-falling ideal accelerometer reads zero and
% mechanical energy is constant. Positive body-x specific force adds work;
% because the body is nose-up, that force also points partly toward negative
% Down and raises the apex. Energy change must equal work, not remain zero.
disp(['Mechanism 1: non-gravity specific force adds frame-invariant power; ' ...
    'mechanical energy changes by exactly the accumulated work.']);

%% Lever 2 - reset specific force and sweep only heading
headingSweep_deg=[-90 -30 0 30 90];
headingNorth_m=zeros(numel(headingSweep_deg),baseline.sampleCount);
headingEast_m=zeros(numel(headingSweep_deg),baseline.sampleCount);
finalNorth_m=zeros(size(headingSweep_deg));
finalEast_m=zeros(size(headingSweep_deg));
for k=1:numel(headingSweep_deg)
    sample=model(1.5,headingSweep_deg(k));
    headingNorth_m(k,:)=sample.positionNED_m(1,:);
    headingEast_m(k,:)=sample.positionNED_m(2,:);
    finalNorth_m(k)=sample.positionNED_m(1,end);
    finalEast_m(k)=sample.positionNED_m(2,end);
    bodyVelocityDifference=sample.velocityBody_mps-baseline.velocityBody_mps;
    workDifference=sample.workInput_J-baseline.workInput_J;
    energyDifference= ...
        sample.mechanicalEnergy_J-baseline.mechanicalEnergy_J;
    brokenDifference=sample.brokenEnergyBalanceResidual_J- ...
        baseline.brokenEnergyBalanceResidual_J;
    assert(isequal(sample.altitude_m,baseline.altitude_m) && ...
        max(abs(bodyVelocityDifference(:)))<1e-12 && ...
        max(abs(workDifference))<1e-7 && ...
        max(abs(energyDifference))<1e-7 && ...
        max(abs(brokenDifference))<1e-7, ...
        ['The heading sweep must rotate North/East geometry while leaving ' ...
        'body velocity, altitude, work, and both energy ledgers fixed.']);
end

%% Changed view - heading actively yaws the path in fixed NED
figure('Name','P12 heading sweep');
subplot(1,2,1);
plot(headingNorth_m.',headingEast_m.','LineWidth',1.3);
grid on; axis equal; xlabel('North (m)'); ylabel('East (m)');
legend(compose('heading %.0f deg',headingSweep_deg),'Location','best');
title('Yawed path family in fixed NED');
subplot(1,2,2);
plot(finalNorth_m,finalEast_m,'s-','LineWidth',1.5); hold on;
plot(0,0,'ko','MarkerFaceColor','k');
grid on; axis equal; xlabel('Final North (m)'); ylabel('Final East (m)');
title('Heading sets signed N/E components');
assert(max(abs(sqrt(finalNorth_m.^2+finalEast_m.^2)- ...
    baseline.horizontalRange_m))<1e-10, ...
    'Heading may rotate horizontal range but must not change its magnitude.');

%% Read and explain lever 2
% Heading is an active yaw of the body and trajectory relative to fixed NED,
% not a passive relabeling of one unchanged vector. The model has uniform
% gravity and no wind or other horizontal asymmetry, so yaw rotates the
% North/East histories while leaving vertical motion and scalar ledgers
% unchanged. A proper DCM separately preserves norms and dot products.
disp(['Mechanism 2: active yaw rotates the path within fixed NED; horizontal ' ...
    'symmetry preserves vertical/scalar histories, and the proper DCM ' ...
    'preserves speed and power dot products.']);

%% Limiting and signed cases - free fall and due-East motion
freeFall=model(0,30);
dueEast=model(1.5,90);
assert(all(freeFall.specificForceBody_mps2==0) && ...
    all(freeFall.specificForceNED_mps2==0) && ...
    all(freeFall.powerBody_W==0) && all(freeFall.workInput_J==0) && ...
    max(abs(freeFall.mechanicalEnergy_J- ...
    freeFall.mechanicalEnergy_J(1)))<1e-7, ...
    ['Zero non-gravity specific force must be the free-fall limit with ' ...
    'constant mechanical energy and zero ideal accelerometer output.']);
assert(abs(dueEast.initialVelocityNED_mps(1))<1e-12 && ...
    dueEast.initialVelocityNED_mps(2)>0 && ...
    dueEast.initialVelocityNED_mps(3)<0 && ...
    abs(dueEast.specificForceNED_mps2(1))<1e-12 && ...
    dueEast.specificForceNED_mps2(2)>0 && ...
    dueEast.specificForceNED_mps2(3)<0, ...
    ['A +90 deg heading must map body-forward velocity and force East while ' ...
    'positive pitch maps both toward negative Down.']);

%% Deliberately broken case - confuse positive Down with positive height
figure('Name','P12 broken Down-as-height energy');
subplot(1,2,1);
plot(baseline.time_s,baseline.energyBalanceResidual_J/1e3, ...
    'LineWidth',1.8); hold on;
plot(baseline.time_s,baseline.brokenEnergyBalanceResidual_J/1e3, ...
    '--','LineWidth',1.8);
grid on; xlabel('Time (s)'); ylabel('Energy balance residual (kJ)');
legend({'correct h=-Down','broken h=+Down'},'Location','best');
title('A sign error creates false unexplained energy');
subplot(1,2,2);
plot(baseline.time_s,baseline.altitude_m-baseline.initialAltitude_m, ...
    'LineWidth',1.7); hold on;
plot(baseline.time_s,baseline.downChange_m,'--','LineWidth',1.7);
grid on; xlabel('Time (s)'); ylabel('Vertical coordinate change (m)');
legend({'altitude change','Down change'},'Location','best');
title('Altitude change is the negative of Down change');
assert(baseline.brokenEnergyBalanceResidual_J(1)==0 && ...
    baseline.maxBrokenResidualClosure_J<1e-7 && ...
    baseline.peakBrokenEnergyDrift_J>1e6 && ...
    isequal(baseline.brokenHeight_m,baseline.positionNED_m(3,:)) && ...
    isequal(baseline.brokenPotentialEnergy_J, ...
    baseline.mass_kg*baseline.gravity_mps2*baseline.positionNED_m(3,:)), ...
    'The broken ledger must isolate the Down-as-height sign error.');
fprintf(['Broken symptom: correct balance stays below %.3g J while the wrong ' ...
    'sign reaches %.3f MJ; both are zero at the initial sample.\n'], ...
    baseline.maxEnergyBalanceResidual_J,baseline.peakBrokenEnergyDrift_J/1e6);

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach-back: state the body-to-NED direction, trace specific force ' ...
    'through power and work, explain h=-Down, then diagnose the broken sign.']);
