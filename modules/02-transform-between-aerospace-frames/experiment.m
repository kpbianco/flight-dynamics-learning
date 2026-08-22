%% P02 - Transform Between Aerospace Frames
% Guiding question:
% What inputs, observable effects, and failure modes matter when you transform Between Aerospace Frames?
% Replace only figures owned by this learning harness; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P[0-9][0-9] ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - name the vector, both frames, and the transform direction
% Body axes are x forward, y right, z down. Navigation axes are North,
% East, Down (NED). A direction cosine matrix changes the coordinates used
% to describe a vector; it does not rotate or resize the physical vector.
% Predict once: with zero sideslip, will changing yaw alter airspeed or only
% the North/East components?
disp('A frame transform changes a vector''s coordinates, not the vector itself.');
disp('Prediction: changing yaw should redirect North/East components without changing speed.');

%% Baseline - express one deterministic velocity in body and NED axes
baseline=model(70,6,0,0,9,30);
fprintf(['Baseline inputs: V=%.1f m/s, alpha=%.1f deg, beta=%.1f deg, ' ...
    'roll=%.1f deg, pitch=%.1f deg, yaw=%.1f deg\n'], ...
    baseline.speed_mps,baseline.alphaDeg,baseline.betaDeg,baseline.rollDeg, ...
    baseline.pitchDeg,baseline.yawDeg);
fprintf('Body [u v w] = [%.2f %.2f %.2f] m/s\n', ...
    baseline.velocityBody_mps(1),baseline.velocityBody_mps(2),baseline.velocityBody_mps(3));
fprintf('NED [N E D] = [%.2f %.2f %.2f] m/s\n', ...
    baseline.velocityNed_mps(1),baseline.velocityNed_mps(2),baseline.velocityNed_mps(3));
fprintf('Air-relative track = %.2f deg; flight-path angle = %.2f deg (positive climb)\n', ...
    baseline.trackDeg,baseline.flightPathDeg);

figure('Name','P02 deterministic baseline');
subplot(1,2,1);
quiver3(0,0,0,baseline.velocityBody_mps(1),baseline.velocityBody_mps(2), ...
    baseline.velocityBody_mps(3),0,'LineWidth',2,'MaxHeadSize',0.25);
grid on; axis equal;
xlabel('x_b forward velocity (m/s)'); ylabel('y_b right velocity (m/s)');
zlabel('z_b down velocity (m/s)'); title('Same vector: body components');
set(gca,'ZDir','reverse'); view(35,24);
bodyLimit=1.1*baseline.speed_mps;
xlim([-bodyLimit bodyLimit]); ylim([-bodyLimit bodyLimit]); zlim([-bodyLimit bodyLimit]);

subplot(1,2,2);
quiver3(0,0,0,baseline.velocityNed_mps(1),baseline.velocityNed_mps(2), ...
    baseline.velocityNed_mps(3),0,'LineWidth',2,'MaxHeadSize',0.25);
grid on; axis equal;
xlabel('North velocity (m/s)'); ylabel('East velocity (m/s)');
zlabel('Down velocity (m/s)'); title('Same vector: NED components');
set(gca,'ZDir','reverse'); view(35,24);
xlim([-bodyLimit bodyLimit]); ylim([-bodyLimit bodyLimit]); zlim([-bodyLimit bodyLimit]);

%% Lever 1 - sweep yaw while every other input stays at baseline
yawSweepDeg=[-60 -15 30 75 120];
yawNed=zeros(3,numel(yawSweepDeg));
yawTrackDeg=zeros(size(yawSweepDeg));
yawFlightPathDeg=zeros(size(yawSweepDeg));
for k=1:numel(yawSweepDeg)
    sample=model(70,6,0,0,9,yawSweepDeg(k));
    yawNed(:,k)=sample.velocityNed_mps;
    yawTrackDeg(k)=sample.trackDeg;
    yawFlightPathDeg(k)=sample.flightPathDeg;
end

%% Changed view - yaw rotates the horizontal components, not the speed
figure('Name','P02 yaw sweep');
subplot(1,2,1); hold on;
for k=1:numel(yawSweepDeg)
    quiver(0,0,yawNed(1,k),yawNed(2,k),0,'LineWidth',1.4, ...
        'DisplayName',sprintf('yaw = %g deg',yawSweepDeg(k)));
end
axis equal; grid on;
xlabel('North velocity (m/s)'); ylabel('East velocity (m/s)');
title('Yaw redirects the horizontal velocity'); legend('Location','best');
subplot(1,2,2);
plot(yawSweepDeg,yawTrackDeg,'o-','LineWidth',1.4,'DisplayName','track'); hold on;
plot(yawSweepDeg,yawFlightPathDeg,'s-','LineWidth',1.4,'DisplayName','flight path');
grid on; xlabel('Yaw angle (deg)'); ylabel('Observed angle (deg)');
title('Track follows yaw; climb angle stays fixed'); legend('Location','best');

%% Read and explain - mechanism for lever 1
disp(['Mechanism: yaw changes the first two rows of C_body_to_ned. ' ...
    'Because an orthonormal DCM preserves length, the N/E split changes while |V| does not.']);

%% Lever 2 - reset yaw, then sweep sideslip independently
betaSweepDeg=[-15 -7.5 0 7.5 15];
bodyLateral_mps=zeros(size(betaSweepDeg));
betaTrackDeg=zeros(size(betaSweepDeg));
betaNormError_mps=zeros(size(betaSweepDeg));
for k=1:numel(betaSweepDeg)
    sample=model(70,6,betaSweepDeg(k),0,9,30);
    bodyLateral_mps(k)=sample.velocityBody_mps(2);
    betaTrackDeg(k)=sample.trackDeg;
    betaNormError_mps(k)=sample.normError_mps;
end

%% Changed view - sideslip appears first as body-axis lateral velocity
figure('Name','P02 sideslip sweep');
subplot(1,2,1);
plot(betaSweepDeg,bodyLateral_mps,'o-','LineWidth',1.4); grid on;
xlabel('Sideslip beta (deg)'); ylabel('Body lateral velocity v (m/s)');
title('v = V sin(beta)');
subplot(1,2,2);
plot(betaSweepDeg,betaTrackDeg,'o-','LineWidth',1.4); grid on;
xlabel('Sideslip beta (deg)'); ylabel('Air-relative track (deg)');
title('Sideslip shifts track away from yaw');

%% Read and explain - mechanism for lever 2
disp(['Mechanism: beta first creates v = V sin(beta) in body y. ' ...
    'The attitude matrix then distributes that component into NED coordinates.']);
fprintf('Largest norm error across the beta sweep: %.3g m/s\n',max(betaNormError_mps));

%% Broken case - transpose the DCM and use the wrong transform direction
rotationOnly=model(70,0,0,0,0,90);
correctNed=rotationOnly.velocityNed_mps;
brokenNed=rotationOnly.C_body_to_ned.'*rotationOnly.velocityBody_mps;
cosSeparation=dot(correctNed,brokenNed)/(norm(correctNed)*norm(brokenNed));
angularErrorDeg=acosd(max(-1,min(1,cosSeparation)));

figure('Name','P02 broken transform direction');
subplot(1,2,1); hold on;
quiver(0,0,correctNed(1),correctNed(2),0,'LineWidth',1.8, ...
    'DisplayName','correct C_{n<-b} v_b');
quiver(0,0,brokenNed(1),brokenNed(2),0,'--','LineWidth',1.8, ...
    'DisplayName','broken C_{n<-b}^T v_b');
axis equal; grid on; xlabel('North velocity (m/s)'); ylabel('East velocity (m/s)');
title('Yaw = 90 deg: east becomes west'); legend('Location','best');
subplot(1,2,2);
bar([correctNed brokenNed]); grid on;
set(gca,'XTickLabel',{'North','East','Down'});
ylabel('Velocity component (m/s)'); title('Correct and broken NED components');
legend('correct','broken','Location','best');

fprintf(['Broken symptom: direction error %.1f deg, but speed still %.1f m/s. ' ...
    'A norm-only check would miss the reversed transform.\n'],angularErrorDeg,norm(brokenNed));
assert(abs(norm(brokenNed)-rotationOnly.speed_mps)<1e-10, ...
    'The deliberately broken orthonormal transform should still preserve vector magnitude.');
assert(angularErrorDeg>179, ...
    'The transpose misuse should reverse the simple yaw-only test vector.');

%% Check and teach back
clear run_checks;
run_checks;
disp('Teach back in two sentences: name the transform direction, then explain why norm checks alone cannot catch every frame error.');
