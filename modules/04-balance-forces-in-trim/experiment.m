%% P04 - Balance Forces in Trim
% Guiding question:
% What inputs, observable effects, and failure modes matter when you balance Forces in Trim?
% Replace only figures owned by this learning harness; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P[0-9][0-9] ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - turn the P03 air state into forces
% P03 supplied air density and true airspeed, so q=0.5*rho*V^2 is known.
% A steady point-mass flight path then needs lift to balance W*cos(gamma)
% and thrust to balance drag plus W*sin(gamma). Predict once: if mass stays
% fixed and true airspeed falls, must the required lift coefficient rise or fall?
disp('Force trim means both the normal and along-path residuals are zero.');
disp('The calculation reports required alpha and thrust without hiding feasibility limits.');

%% Baseline - one deterministic steady, level force balance
baseline=model(0.736115547399152,60,1200,0);
fprintf(['Baseline inputs: P03 density=%.5f kg/m^3, true airspeed=%.1f m/s, ' ...
    'mass=%.0f kg, flight-path angle=%+.1f deg\n'],baseline.airDensity_kgpm3, ...
    baseline.trueAirspeed_mps,baseline.mass_kg,baseline.flightPathAngle_deg);
fprintf(['q=%.2f Pa; W=%.2f N; CL=%.4f; alpha=%.2f deg; ' ...
    'drag=%.2f N; thrust=%.2f N\n'],baseline.dynamicPressure_Pa, ...
    baseline.weight_N,baseline.liftCoefficient,baseline.angleOfAttack_deg, ...
    baseline.drag_N,baseline.thrustRequired_N);
fprintf(['stall speed=%.2f m/s; minimum-drag speed=%.2f m/s; ' ...
    'force residual=%.3g N; feasible=%d\n'],baseline.stallSpeed_mps, ...
    baseline.minimumDragSpeed_mps,baseline.forceResidualMagnitude_N, ...
    baseline.trimFeasible);

figure('Name','P04 deterministic baseline');
subplot(1,2,1);
forcePairs_N=[baseline.lift_N -baseline.normalForceRequired_N; ...
    baseline.thrustRequired_N -(baseline.drag_N+baseline.weightAlongPath_N)];
bar(forcePairs_N); grid on;
set(gca,'XTickLabel',{'normal balance','along-path balance'});
ylabel('Force (N)');
legend({'positive-axis force','opposing force'},'Location','best');
title('Opposing forces sum to zero');
subplot(1,2,2);
bar([baseline.parasiteDrag_N baseline.inducedDrag_N]); grid on;
set(gca,'XTickLabel',{'parasite drag','induced drag'});
ylabel('Drag contribution (N)');
title(sprintf('Total drag = required thrust = %.1f N',baseline.drag_N));

%% Lever 1 - sweep true airspeed with density, mass, and path fixed
speedSweep_mps=[40 50 60 80 100];
speedLiftCoefficient=zeros(size(speedSweep_mps));
speedAngleOfAttack_deg=zeros(size(speedSweep_mps));
speedParasiteDrag_N=zeros(size(speedSweep_mps));
speedInducedDrag_N=zeros(size(speedSweep_mps));
speedThrustRequired_N=zeros(size(speedSweep_mps));
speedTrimFeasible=false(size(speedSweep_mps));
for k=1:numel(speedSweep_mps)
    sample=model(0.736115547399152,speedSweep_mps(k),1200,0);
    speedLiftCoefficient(k)=sample.liftCoefficient;
    speedAngleOfAttack_deg(k)=sample.angleOfAttack_deg;
    speedParasiteDrag_N(k)=sample.parasiteDrag_N;
    speedInducedDrag_N(k)=sample.inducedDrag_N;
    speedThrustRequired_N(k)=sample.thrustRequired_N;
    speedTrimFeasible(k)=sample.trimFeasible;
end

%% Changed view - speed trades lift coefficient against two kinds of drag
figure('Name','P04 true-airspeed sweep');
subplot(1,2,1);
yyaxis left;
plot(speedSweep_mps,speedLiftCoefficient,'o-','LineWidth',1.4);
ylabel('Required lift coefficient C_L (-)');
yyaxis right;
plot(speedSweep_mps,speedAngleOfAttack_deg,'s-','LineWidth',1.4);
ylabel('Required angle of attack (deg)');
grid on; xlabel('True airspeed (m/s)');
title('Less q requires more C_L and alpha');
subplot(1,2,2);
plot(speedSweep_mps,speedParasiteDrag_N,'o-','LineWidth',1.4); hold on;
plot(speedSweep_mps,speedInducedDrag_N,'s-','LineWidth',1.4);
plot(speedSweep_mps,speedThrustRequired_N,'d-','LineWidth',1.6);
grid on; xlabel('True airspeed (m/s)'); ylabel('Force (N)');
legend({'parasite drag','induced drag','required thrust'},'Location','best');
title('The drag trade produces a minimum-thrust speed');
fprintf('All %d airspeed-sweep points feasible: %d\n', ...
    numel(speedSweep_mps),all(speedTrimFeasible));

%% Read and explain - mechanism for lever 1
disp(['Mechanism: fixed lift with lower q needs larger CL and alpha. ' ...
    'Parasite drag scales with V^2, while induced drag scales with 1/V^2 at fixed lift.']);
disp('Required thrust follows their sum; a plausible alpha alone does not prove both forces balance.');

%% Lever 2 - reset speed, then sweep mass independently
massSweep_kg=[800 1000 1200 1400 1600];
massLiftCoefficient=zeros(size(massSweep_kg));
massAngleOfAttack_deg=zeros(size(massSweep_kg));
massParasiteDrag_N=zeros(size(massSweep_kg));
massInducedDrag_N=zeros(size(massSweep_kg));
massThrustRequired_N=zeros(size(massSweep_kg));
for k=1:numel(massSweep_kg)
    sample=model(0.736115547399152,60,massSweep_kg(k),0);
    massLiftCoefficient(k)=sample.liftCoefficient;
    massAngleOfAttack_deg(k)=sample.angleOfAttack_deg;
    massParasiteDrag_N(k)=sample.parasiteDrag_N;
    massInducedDrag_N(k)=sample.inducedDrag_N;
    massThrustRequired_N(k)=sample.thrustRequired_N;
end

%% Changed view - added mass raises lift demand and induced drag
figure('Name','P04 mass sweep');
subplot(1,2,1);
yyaxis left;
plot(massSweep_kg,massLiftCoefficient,'o-','LineWidth',1.4);
ylabel('Required lift coefficient C_L (-)');
yyaxis right;
plot(massSweep_kg,massAngleOfAttack_deg,'s-','LineWidth',1.4);
ylabel('Required angle of attack (deg)');
grid on; xlabel('Aircraft mass (kg)');
title('More weight requires more lift at fixed q');
subplot(1,2,2);
plot(massSweep_kg,massParasiteDrag_N,'o-','LineWidth',1.4); hold on;
plot(massSweep_kg,massInducedDrag_N,'s-','LineWidth',1.4);
plot(massSweep_kg,massThrustRequired_N,'d-','LineWidth',1.6);
grid on; xlabel('Aircraft mass (kg)'); ylabel('Force (N)');
legend({'parasite drag','induced drag','required thrust'},'Location','best');
title('Parasite drag stays fixed; induced drag grows');

%% Read and explain - mechanism for lever 2
disp(['Mechanism: mass raises W=m*g. At unchanged density and speed, q is fixed, ' ...
    'so CL, alpha, and induced drag must rise while parasite drag stays fixed.']);

%% Broken case - omit the one-half when turning rho and V into q
% The broken coefficient uses rho*V^2 instead of q=0.5*rho*V^2. It looks
% numerically tidy but commands only half the lift needed for level flight.
brokenLiftCoefficient=baseline.weight_N/(baseline.airDensity_kgpm3* ...
    baseline.trueAirspeed_mps^2*baseline.wingArea_m2);
brokenLift_N=baseline.dynamicPressure_Pa*baseline.wingArea_m2* ...
    brokenLiftCoefficient;
brokenNormalResidual_N=brokenLift_N-baseline.normalForceRequired_N;

figure('Name','P04 broken dynamic-pressure factor');
subplot(1,2,1);
bar([baseline.normalForceRequired_N baseline.lift_N brokenLift_N]); grid on;
set(gca,'XTickLabel',{'required','correct lift','broken lift'});
ylabel('Normal force (N)');
title('Missing 0.5 commands only half the lift');
subplot(1,2,2);
bar([baseline.normalForceResidual_N brokenNormalResidual_N]); grid on;
set(gca,'XTickLabel',{'correct residual','broken residual'});
ylabel('Normal force residual (N)');
title('A coefficient is not trim until residuals close');
fprintf(['Broken symptom: missing the 0.5 in q leaves %.2f N normal residual ' ...
    '(%.1f%% of required lift).\n'],brokenNormalResidual_N, ...
    100*brokenNormalResidual_N/baseline.normalForceRequired_N);
assert(abs(brokenLift_N/baseline.normalForceRequired_N-0.5)<1e-12, ...
    'The deliberately broken q calculation must command exactly half the required lift.');

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach back in two sentences: connect P03 density and true airspeed through q to lift and drag; ' ...
    'then explain why both force residuals and feasibility margins must be checked before calling a state trim.']);
