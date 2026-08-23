%% P18 - Follow Waypoints
% Guiding question:
% What inputs, observable effects, and failure modes matter when you follow Waypoints?
%
% Predict once: if the waypoint arrival radius grows while the route and
% course response stay fixed, will the recorded path get longer or shorter?

%% Baseline - read the route, then follow one active waypoint at a time
baseline=model(30,0.8,1);
fprintf(['Baseline: %d of %d target waypoints captured in %.1f s; ' ...
    'flown distance %.1f m; course-error RMS %.3f deg.\n'], ...
    baseline.targetWaypointCapturedCount,baseline.waypointCount-1, ...
    baseline.missionCompletionTime_s,baseline.flownDistance_m, ...
    baseline.courseErrorRMS_deg);

activePath=baseline.motionActive;
figure('Name','P18 Baseline Waypoint Route');
plot(baseline.waypointEast_m,baseline.waypointNorth_m,'k--o', ...
    'LineWidth',1.4,'MarkerFaceColor','k');
hold on;
plot(baseline.eastPosition_m(activePath), ...
    baseline.northPosition_m(activePath),'LineWidth',1.8);
plot(baseline.waypointCaptureEast_m(baseline.waypointCaptured), ...
    baseline.waypointCaptureNorth_m(baseline.waypointCaptured),'s', ...
    'MarkerSize',9,'LineWidth',1.5);
circleAngle_rad=linspace(0,2*pi,181);
for waypointIndex=2:baseline.waypointCount
    plot(baseline.waypointEast_m(waypointIndex)+ ...
        baseline.arrivalRadius_m*cos(circleAngle_rad), ...
        baseline.waypointNorth_m(waypointIndex)+ ...
        baseline.arrivalRadius_m*sin(circleAngle_rad),':', ...
        'Color',[0.55 0.55 0.55]);
end
grid on; axis equal;
xlabel('East position (m)'); ylabel('North position (m)');
legend({'planned legs','flown path','capture samples','arrival circles'}, ...
    'Location','best');
title('Ordered stationary waypoints and inclusive arrival circles');

%% Changed view - expose bearing, shortest error, and bounded response
figure('Name','P18 Baseline Guidance Processing');
subplot(3,1,1);
plot(baseline.time_s(activePath), ...
    baseline.courseCommand_deg(activePath),'LineWidth',1.5);
hold on;
plot(baseline.time_s(activePath),baseline.course_deg(activePath), ...
    'LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('Course (deg)');
legend({'commanded bearing','actual course'},'Location','best');
title('atan2(Delta East,Delta North) commands the active waypoint');

subplot(3,1,2);
plot(baseline.time_s(activePath), ...
    baseline.courseError_deg(activePath),'LineWidth',1.5);
hold on;
plot(baseline.time_s(activePath), ...
    baseline.courseRate_degps(activePath),'LineWidth',1.3);
grid on;
xlabel('Time (s)'); ylabel('Error (deg) or rate (deg/s)');
legend({'shortest course error','bounded course rate'}, ...
    'Location','best');
title('The +/-12 deg/s rate limit makes corner response visible');

subplot(3,1,3);
stairs(baseline.time_s(activePath), ...
    100*baseline.activeWaypointIndex(activePath),'LineWidth',1.5);
hold on;
plot(baseline.time_s(activePath), ...
    baseline.rangeToActiveWaypoint_m(activePath),'LineWidth',1.2);
plot(baseline.time_s(activePath), ...
    baseline.arrivalRadius_m*ones(1,sum(activePath)),'k--');
grid on;
xlabel('Time (s)'); ylabel('Range (m) or 100 x waypoint index (-)');
legend({'100 x active waypoint index','range to active waypoint', ...
    'arrival radius'},'Location','best');
title('Sequencing changes only after an inclusive 2-D range test');

%% Lever 1 - sweep arrival radius with response gain reset
arrivalRadiusSweep_m=[10 20 30 50 80];
radiusCompletionTime_s=zeros(size(arrivalRadiusSweep_m));
radiusFlownDistance_m=zeros(size(arrivalRadiusSweep_m));
radiusCaptureRange_m=zeros(size(arrivalRadiusSweep_m));

figure('Name','P18 Arrival Radius Sweep');
subplot(2,1,1); hold on;
for k=1:numel(arrivalRadiusSweep_m)
    sample=model(arrivalRadiusSweep_m(k), ...
        baseline.courseResponseGain_per_s,1);
    moving=sample.motionActive;
    plot(sample.eastPosition_m(moving),sample.northPosition_m(moving), ...
        'LineWidth',1.2);
    radiusCompletionTime_s(k)=sample.missionCompletionTime_s;
    radiusFlownDistance_m(k)=sample.flownDistance_m;
    radiusCaptureRange_m(k)=sample.waypointCaptureRange_m(end);
end
plot(baseline.waypointEast_m,baseline.waypointNorth_m,'k--o', ...
    'LineWidth',1.2,'MarkerFaceColor','k');
grid on; axis equal;
xlabel('East position (m)'); ylabel('North position (m)');
radiusLegend=cellstr(compose('R = %.0f m',arrivalRadiusSweep_m));
radiusLegend{end+1}='planned legs';
legend(radiusLegend,'Location','best');
title('Lever 1: a larger arrival circle switches earlier and cuts corners');

subplot(2,1,2);
yyaxis left;
plot(arrivalRadiusSweep_m,radiusFlownDistance_m,'o-', ...
    'LineWidth',1.5);
ylabel('Flown distance (m)');
yyaxis right;
plot(arrivalRadiusSweep_m,radiusCaptureRange_m,'s-', ...
    'LineWidth',1.5);
ylabel('Final capture range (m)');
grid on;
xlabel('Waypoint arrival radius (m)');
legend({'flown distance','final capture range'},'Location','best');
title('Mechanism: earlier switching trades closeness for a shorter path');

%% Lever 2 - reset radius, then sweep course-response gain
courseResponseGainSweep_per_s=[0 0.2 0.4 0.8 1.2];
gainCompletionTime_s=zeros(size(courseResponseGainSweep_per_s));
gainCapturedCount=zeros(size(courseResponseGainSweep_per_s));
gainSaturationFraction=zeros(size(courseResponseGainSweep_per_s));
gainPeakCourseError_deg=zeros(size(courseResponseGainSweep_per_s));

figure('Name','P18 Course Response Gain Sweep');
subplot(2,1,1); hold on;
for k=1:numel(courseResponseGainSweep_per_s)
    sample=model(baseline.arrivalRadius_m, ...
        courseResponseGainSweep_per_s(k),1);
    moving=sample.motionActive;
    plot(sample.eastPosition_m(moving),sample.northPosition_m(moving), ...
        'LineWidth',1.2);
    gainCompletionTime_s(k)=sample.missionCompletionTime_s;
    gainCapturedCount(k)=sample.targetWaypointCapturedCount;
    gainSaturationFraction(k)=sample.courseRateSaturationFraction;
    gainPeakCourseError_deg(k)=sample.peakAbsoluteCourseError_deg;
end
plot(baseline.waypointEast_m,baseline.waypointNorth_m,'k--o', ...
    'LineWidth',1.2,'MarkerFaceColor','k');
grid on; axis equal;
xlabel('East position (m)'); ylabel('North position (m)');
gainLegend=cellstr(compose('K_{chi} = %.1f 1/s', ...
    courseResponseGainSweep_per_s));
gainLegend{end+1}='planned legs';
legend(gainLegend,'Location','best');
title('Lever 2: response gain changes turn capture, not arrival geometry');

subplot(2,1,2);
plot(courseResponseGainSweep_per_s,gainCapturedCount,'o-', ...
    'LineWidth',1.5);
hold on;
plot(courseResponseGainSweep_per_s, ...
    10*gainSaturationFraction,'s-','LineWidth',1.5);
grid on;
xlabel('Course-response gain (1/s)');
ylabel('Captured targets or 10 x saturation fraction (-)');
legend({'captured target waypoints','10 x rate saturation fraction'}, ...
    'Location','best');
title('Mechanism: gain helps only until the fixed rate cap dominates');

%% Limiting case - zero response cannot turn after the first waypoint
zeroResponse=model(baseline.arrivalRadius_m,0,1);
fprintf(['Zero-gain limit: %d of %d target waypoints captured; ' ...
    'course-rate peak %.3g deg/s; route complete = %d.\n'], ...
    zeroResponse.targetWaypointCapturedCount, ...
    zeroResponse.waypointCount-1,max(abs(zeroResponse.courseRate_degps)), ...
    zeroResponse.routeComplete);

%% Deliberately broken - swap North and East in the bearing calculation
broken=model(baseline.arrivalRadius_m, ...
    baseline.courseResponseGain_per_s,-1);
fprintf(['Broken bearing: first command %.1f deg instead of %.1f deg; ' ...
    '%d target waypoints captured; closest first-target range %.1f m.\n'], ...
    broken.courseCommand_deg(1),baseline.courseCommand_deg(1), ...
    broken.targetWaypointCapturedCount, ...
    broken.minimumRangeToWaypoint_m(2));

figure('Name','P18 Broken North East Bearing');
subplot(2,1,1);
plot(baseline.waypointEast_m,baseline.waypointNorth_m,'k--o', ...
    'LineWidth',1.4,'MarkerFaceColor','k');
hold on;
plot(baseline.eastPosition_m(baseline.motionActive), ...
    baseline.northPosition_m(baseline.motionActive),'LineWidth',1.7);
plot(broken.eastPosition_m(broken.motionActive), ...
    broken.northPosition_m(broken.motionActive),'--','LineWidth',1.7);
grid on; axis equal;
xlabel('East position (m)'); ylabel('North position (m)');
legend({'planned legs','correct N/E bearing','BROKEN swapped bearing'}, ...
    'Location','best');
title('Same route and response; only atan2 argument order changes');

subplot(2,1,2);
comparisonWindow=baseline.time_s<=10;
plot(baseline.time_s(comparisonWindow), ...
    baseline.courseCommand_deg(comparisonWindow),'LineWidth',1.5);
hold on;
plot(broken.time_s(comparisonWindow), ...
    broken.courseCommand_deg(comparisonWindow),'--','LineWidth',1.5);
plot(broken.time_s(comparisonWindow), ...
    broken.course_deg(comparisonWindow),':','LineWidth',1.5);
grid on;
xlabel('Time (s)'); ylabel('Course (deg)');
legend({'correct command','BROKEN command','BROKEN response'}, ...
    'Location','best');
title('A due-North target is incorrectly interpreted as East');

%% Explain, check, and teach back
% Reset to model(30,0.8,1). An ordered route supplies the active stationary
% waypoint. Its North/East displacement becomes a clockwise-from-North
% bearing, shortest circular error drives a bounded course response, and an
% inclusive Euclidean arrival circle advances the route. P17 supplies the
% navigation-estimate idea, not runtime arrays; P19 will add moving-target
% pursuit, which this fixed-route model does not implement.
clear run_checks;
run_checks;
