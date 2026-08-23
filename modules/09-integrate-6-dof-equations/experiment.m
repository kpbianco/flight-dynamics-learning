%% P09 - Integrate 6-DOF Equations
% Guiding question:
% What inputs, observable effects, and failure modes matter when you integrate 6-DOF Equations?
% Replace only figures owned by this module; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P09 ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - assemble translation, rotation, attitude, and position
% P08 stopped after a four-state lateral perturbation model. P09 propagates
% NED position, body velocity, a body-to-NED quaternion, and body rates.
% Applied body force excludes gravity. The body-axis velocity equation must
% include -omega cross v because its components rotate with the aircraft.
disp('P08 produced force and moment tendencies; P09 advances the complete rigid-body state.');
disp(['Predict once: after a positive three-axis moment pulse, can body velocity ' ...
    'components stay constant while the vehicle rotates, even if inertial velocity does not?']);

%% Baseline - apply one deterministic force pulse and one moment pulse
baseline=model(1,1);
fprintf(['Baseline inputs: forward-force scale %.2f, moment scale %.2f, ' ...
    'step %.3f s, horizon %.1f s, %d samples.\n'], ...
    baseline.forcePulseScale,baseline.momentPulseScale, ...
    baseline.integrationStep_s,baseline.timeHorizon_s,baseline.sampleCount);
fprintf(['Rigid body: m=%.0f kg, I=[%.0f %.0f %.0f] kg*m^2, ' ...
    'u(0)=%.1f m/s, gravity=%.5f m/s^2.\n'],baseline.mass_kg, ...
    baseline.inertiaTensor_kgm2(1,1),baseline.inertiaTensor_kgm2(2,2), ...
    baseline.inertiaTensor_kgm2(3,3),baseline.initialForwardSpeed_mps, ...
    baseline.gravity_mps2);
fprintf(['Final NED position: north %.3f m, east %.3f m, down %.3f m; ' ...
    'final inertial speed %.3f m/s.\n'],baseline.finalPositionNED_m(1), ...
    baseline.finalPositionNED_m(2),baseline.finalPositionNED_m(3), ...
    baseline.finalSpeed_mps);
fprintf(['Peak body-rate magnitude %.3f deg/s; peak attitude rotation %.3f deg; ' ...
    'post-pulse inertial angular-momentum drift %.3g relative.\n'], ...
    baseline.peakBodyRateMagnitude_deg_s, ...
    baseline.peakAttitudeRotation_deg, ...
    baseline.postPulseAngularMomentumRelativeDrift);
assert(max(abs(baseline.translationalEquationResidual_mps2(:)))<1e-12 && ...
    max(abs(baseline.rotationalEquationResidual_Nm(:)))<1e-10 && ...
    max(abs(baseline.quaternionNorm-1))<1e-14 && ...
    max(baseline.dcmOrthonormalityError)<1e-14, ...
    'The baseline must close its equations, unit quaternion, and DCM.');

%% Baseline view 1 - position is inertial even when components are integrated in body axes
figure('Name','P09 deterministic NED trajectory');
subplot(1,2,1);
plot(baseline.north_m,baseline.east_m,'LineWidth',1.6); hold on;
plot(baseline.north_m(1),baseline.east_m(1),'ko', ...
    'MarkerFaceColor','k');
grid on; axis equal;
xlabel('North position (m)'); ylabel('East position (m)');
title('Horizontal path from C_{NB} v_b');
subplot(1,2,2);
plot(baseline.time_s,baseline.down_m,'LineWidth',1.6); hold on;
plot(baseline.time_s,zeros(size(baseline.time_s)),'k--');
grid on; xlabel('Time (s)'); ylabel('Down position (m)');
title('NED Down is positive');

%% Baseline view 2 - inspect velocity, body rate, then derived attitude
figure('Name','P09 coupled rigid-body states');
subplot(1,3,1);
plot(baseline.time_s,baseline.velocityBody_mps,'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Body velocity (m/s)');
legend({'u forward','v right','w down'},'Location','best');
title('Rotating body components');
subplot(1,3,2);
plot(baseline.time_s,baseline.bodyRates_deg_s,'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Body rate (deg/s)');
legend({'p roll','q pitch','r yaw'},'Location','best');
title('Euler rotational equation');
subplot(1,3,3);
plot(baseline.time_s,baseline.eulerAngles_deg,'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Derived 3-2-1 angle (deg)');
legend({'roll phi','pitch theta','yaw psi'},'Location','best');
title('Display angles from q_{NB}');

%% Baseline load view - force and moment stay separate from propagation
figure('Name','P09 prescribed body load ledger');
subplot(1,2,1);
plot(baseline.time_s,baseline.appliedForceBody_N,'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Non-gravity body force (N)');
legend({'F_x','F_y','F_z'},'Location','best');
title('Forward pulse plus level weight balance');
subplot(1,2,2);
plot(baseline.time_s,baseline.appliedMomentBody_Nm,'LineWidth',1.3);
grid on; xlabel('Time (s)'); ylabel('Applied body moment (N*m)');
legend({'L roll','M pitch','N yaw'},'Location','best');
title('One finite three-axis moment pulse');

%% Lever 1 - reset moment scale and sweep only the forward-force pulse
forcePulseScaleSweep=[0 0.5 1 1.25 1.5];
northSweep_m=zeros(numel(forcePulseScaleSweep),baseline.sampleCount);
finalNorthSweep_m=zeros(size(forcePulseScaleSweep));
finalSpeedSweep_mps=zeros(size(forcePulseScaleSweep));
for k=1:numel(forcePulseScaleSweep)
    sample=model(forcePulseScaleSweep(k),1);
    northSweep_m(k,:)=sample.north_m;
    finalNorthSweep_m(k)=sample.finalPositionNED_m(1);
    finalSpeedSweep_mps(k)=sample.finalSpeed_mps;
    forceDifference=sample.appliedForceBody_N-baseline.appliedForceBody_N;
    forceDifference(1,:)=0;
    assert(all(forceDifference(:)==0) && ...
        isequal(sample.appliedMomentBody_Nm,baseline.appliedMomentBody_Nm) && ...
        isequal(sample.quaternionBodyToNED,baseline.quaternionBodyToNED) && ...
        isequal(sample.bodyRates_rad_s,baseline.bodyRates_rad_s), ...
        'The force sweep must change only forward force and translation.');
end

%% Changed view - forward impulse advances range without changing rotation
figure('Name','P09 forward-force pulse sweep');
subplot(1,2,1);
plot(baseline.time_s,northSweep_m,'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('North position (m)');
legend(compose('force scale %.2f',forcePulseScaleSweep), ...
    'Location','best');
title('Only the forward pulse changes');
subplot(1,2,2);
plot(forcePulseScaleSweep,finalNorthSweep_m,'o-','LineWidth',1.4); hold on;
plot(forcePulseScaleSweep,finalSpeedSweep_mps,'s-','LineWidth',1.4);
grid on; xlabel('Forward-force pulse scale (-)');
ylabel('Final north (m) or speed (m/s)');
legend({'final north (m)','final speed (m/s)'},'Location','best');
title('Impulse accumulates velocity and range');
assert(all(diff(finalNorthSweep_m)>0) && all(diff(finalSpeedSweep_mps)>0), ...
    'More forward-force impulse must increase final north and speed.');

%% Read and explain - mechanism for lever 1
disp(['Mechanism: scale multiplies only F_x(t). Force divided by mass changes ' ...
    'body acceleration; C_NB then maps the updated body velocity into NED position. ' ...
    'The prescribed moments, quaternion, and body rates remain identical.']);

%% Lever 2 - reset force scale and sweep only the moment pulse
momentPulseScaleSweep=[0 0.5 1 1.25 1.5];
bodyRateSweep_deg_s=zeros(numel(momentPulseScaleSweep),baseline.sampleCount);
peakBodyRateSweep_deg_s=zeros(size(momentPulseScaleSweep));
peakAttitudeSweep_deg=zeros(size(momentPulseScaleSweep));
finalEastSweep_m=zeros(size(momentPulseScaleSweep));
finalDownSweep_m=zeros(size(momentPulseScaleSweep));
for k=1:numel(momentPulseScaleSweep)
    sample=model(1,momentPulseScaleSweep(k));
    bodyRateSweep_deg_s(k,:)=sample.bodyRateMagnitude_deg_s;
    peakBodyRateSweep_deg_s(k)=sample.peakBodyRateMagnitude_deg_s;
    peakAttitudeSweep_deg(k)=sample.peakAttitudeRotation_deg;
    finalEastSweep_m(k)=sample.finalPositionNED_m(2);
    finalDownSweep_m(k)=sample.finalPositionNED_m(3);
    assert(isequal(sample.appliedForceBody_N,baseline.appliedForceBody_N) && ...
        max(abs(sample.appliedMomentBody_Nm(:)- ...
        momentPulseScaleSweep(k)*baseline.appliedMomentBody_Nm(:)))<1e-12, ...
        'The moment sweep must reset force and scale only the moment pulse.');
end

%% Changed view - angular impulse rotates lift and bends the trajectory
figure('Name','P09 moment pulse sweep');
subplot(1,3,1);
plot(baseline.time_s,bodyRateSweep_deg_s,'LineWidth',1.2);
grid on; xlabel('Time (s)'); ylabel('Body-rate magnitude (deg/s)');
legend(compose('moment scale %.2f',momentPulseScaleSweep), ...
    'Location','best');
title('Angular impulse changes body rate');
subplot(1,3,2);
plot(momentPulseScaleSweep,peakAttitudeSweep_deg,'o-','LineWidth',1.4);
grid on; xlabel('Moment pulse scale (-)');
ylabel('Peak attitude rotation (deg)');
title('Quaternion carries accumulated attitude');
subplot(1,3,3);
plot(momentPulseScaleSweep,finalEastSweep_m,'s-','LineWidth',1.4); hold on;
plot(momentPulseScaleSweep,finalDownSweep_m,'d-','LineWidth',1.4);
grid on; xlabel('Moment pulse scale (-)');
ylabel('Final NED displacement (m)');
legend({'east','down'},'Location','best');
title('Rotated lift couples attitude into path');
assert(peakBodyRateSweep_deg_s(1)==0 && ...
    all(diff(peakBodyRateSweep_deg_s)>0) && ...
    all(diff(peakAttitudeSweep_deg)>0) && ...
    finalEastSweep_m(1)==0 && finalDownSweep_m(1)==0, ...
    'The moment sweep must expose zero and increasing rotational responses.');

%% Read and explain - mechanism for lever 2
disp(['Mechanism: moment changes angular momentum through ' ...
    'I*omega-dot + omega cross (I*omega) = M. The quaternion integrates ' ...
    'body rate, and C_NB rotates both velocity and the body-fixed lift force, ' ...
    'so a rotational lever eventually changes the NED path.']);

%% Broken case - omit the rotating-body transport acceleration
% The wrong propagation keeps the same force, moment, quaternion, and body
% rates, but removes -omega cross v from v-dot_body. Its path remains smooth
% even though the complete translational equation leaves a large residual.
figure('Name','P09 broken rotating-frame transport term');
subplot(1,2,1);
plot(baseline.north_m,baseline.east_m,'LineWidth',1.6); hold on;
plot(baseline.brokenPositionNED_m(1,:), ...
    baseline.brokenPositionNED_m(2,:),'--','LineWidth',1.6);
grid on; axis equal;
xlabel('North position (m)'); ylabel('East position (m)');
legend({'complete equation','omit -omega cross v'},'Location','best');
title('A smooth path can violate the frame equation');
subplot(1,2,2);
plot(baseline.time_s,baseline.brokenPositionError_m,'LineWidth',1.6); hold on;
plot(baseline.time_s, ...
    baseline.brokenTransportResidualMagnitude_mps2,'--','LineWidth',1.6);
grid on; xlabel('Time (s)');
ylabel('Position error (m) or closure residual (m/s^2)');
legend({'position separation','translation residual'},'Location','best');
title('Missing transport term leaves a measurable residual');
fprintf(['Broken transport: final position separation %.3f m; peak complete-' ...
    'equation residual %.3f m/s^2. Loads and attitude are unchanged.\n'], ...
    baseline.brokenFinalPositionError_m, ...
    max(baseline.brokenTransportResidualMagnitude_mps2));
assert(baseline.brokenFinalPositionError_m>100 && ...
    max(baseline.brokenTransportResidualMagnitude_mps2)>5 && ...
    isequal(baseline.brokenState(7:13,:),baseline.state(7:13,:)), ...
    'The omitted transport term must isolate a recognizable frame failure.');

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach back in two sentences: first trace body force and moment through ' ...
    'velocity, rate, quaternion, and NED position; then explain why omitting ' ...
    '-omega cross v creates a smooth but physically inconsistent trajectory.']);
