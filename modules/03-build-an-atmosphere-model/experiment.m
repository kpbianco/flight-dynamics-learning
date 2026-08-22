%% P03 - Build an Atmosphere Model
% Guiding question:
% What inputs, observable effects, and failure modes matter when you build an Atmosphere Model?
% Replace only figures owned by this learning harness; preserve unrelated work.
lessonFigures=findall(groot,'Type','figure','-regexp','Name','^P[0-9][0-9] ');
if ~isempty(lessonFigures)
    close(lessonFigures);
end
clc; clear model;

%% Read - connect air-relative speed to the surrounding air
% P02 expressed one air-relative velocity in wind, body, and NED axes. P03
% takes that vector's magnitude as true airspeed and adds the local air
% state. Pressure altitude h is positive up, not NED Down. Predict once:
% at fixed true airspeed, will climbing increase or decrease dynamic pressure?
disp('An atmosphere model maps pressure altitude and temperature departure to p, T, rho, and a.');
disp('Those properties turn P02 true airspeed into Mach, dynamic pressure, and equivalent airspeed.');

%% Baseline - compute one deterministic local atmosphere and flight state
baseline=model(5000,0,150);
fprintf(['Baseline inputs: pressure altitude=%.0f m, temperature offset=%+.1f K, ' ...
    'true airspeed=%.1f m/s\n'],baseline.pressureAltitude_m, ...
    baseline.temperatureOffset_K,baseline.trueAirspeed_mps);
fprintf('Layer: %s; T=%.2f K; p=%.2f kPa; rho=%.4f kg/m^3\n', ...
    baseline.layer,baseline.temperature_K,baseline.pressure_Pa/1000, ...
    baseline.density_kgpm3);
fprintf('a=%.2f m/s; Mach=%.3f; q=%.2f kPa; EAS=%.2f m/s\n', ...
    baseline.speedOfSound_mps,baseline.mach,baseline.dynamicPressure_Pa/1000, ...
    baseline.equivalentAirspeed_mps);

baselineSpeedGrid_mps=[0 50 100 150 200 250 300];
baselineDynamicPressure_kPa=0.5*baseline.density_kgpm3* ...
    baselineSpeedGrid_mps.^2/1000;
figure('Name','P03 deterministic baseline');
subplot(1,2,1);
bar([baseline.temperatureRatio baseline.pressureRatio baseline.densityRatio]);
grid on; set(gca,'XTickLabel',{'T/T_0','p/p_0','rho/rho_0'});
ylabel('Ratio to sea-level reference (-)');
title('Atmospheric state at 5 km pressure altitude');
subplot(1,2,2);
plot(baselineSpeedGrid_mps,baselineDynamicPressure_kPa,'o-','LineWidth',1.4); hold on;
plot(baseline.trueAirspeed_mps,baseline.dynamicPressure_Pa/1000,'o', ...
    'MarkerSize',9,'LineWidth',2,'DisplayName','baseline');
grid on; xlabel('True airspeed (m/s)'); ylabel('Dynamic pressure q (kPa)');
title(sprintf('Local density turns speed into q; Mach = %.3f',baseline.mach));

%% Lever 1 - sweep pressure altitude with weather and airspeed fixed
altitudeSweep_m=[0 3000 6000 9000 11000 15000 20000];
altitudeTemperature_K=zeros(size(altitudeSweep_m));
altitudePressure_kPa=zeros(size(altitudeSweep_m));
altitudeDensity_kgpm3=zeros(size(altitudeSweep_m));
altitudeDynamicPressure_kPa=zeros(size(altitudeSweep_m));
altitudeMach=zeros(size(altitudeSweep_m));
for k=1:numel(altitudeSweep_m)
    sample=model(altitudeSweep_m(k),0,150);
    altitudeTemperature_K(k)=sample.temperature_K;
    altitudePressure_kPa(k)=sample.pressure_Pa/1000;
    altitudeDensity_kgpm3(k)=sample.density_kgpm3;
    altitudeDynamicPressure_kPa(k)=sample.dynamicPressure_Pa/1000;
    altitudeMach(k)=sample.mach;
end

%% Changed view - altitude reshapes the atmosphere and air-data observables
figure('Name','P03 altitude sweep');
subplot(2,2,1);
plot(altitudeSweep_m/1000,altitudeTemperature_K,'o-','LineWidth',1.4); grid on;
xlabel('Pressure altitude (km)'); ylabel('Temperature (K)');
title('Lapse, then isothermal layer');
subplot(2,2,2);
semilogy(altitudeSweep_m/1000,altitudePressure_kPa,'o-','LineWidth',1.4); grid on;
xlabel('Pressure altitude (km)'); ylabel('Static pressure (kPa)');
title('Hydrostatic pressure decrease');
subplot(2,2,3);
plot(altitudeSweep_m/1000,altitudeDensity_kgpm3,'o-','LineWidth',1.4); grid on;
xlabel('Pressure altitude (km)'); ylabel('Density (kg/m^3)');
title('Density follows p/(R T)');
subplot(2,2,4);
yyaxis left;
plot(altitudeSweep_m/1000,altitudeDynamicPressure_kPa,'o-','LineWidth',1.4);
ylabel('Dynamic pressure q (kPa)');
yyaxis right;
plot(altitudeSweep_m/1000,altitudeMach,'s-','LineWidth',1.4);
ylabel('Mach number (-)'); grid on; xlabel('Pressure altitude (km)');
title('Fixed 150 m/s: q falls while Mach rises');

%% Read and explain - mechanism for lever 1
disp(['Mechanism: hydrostatic balance lowers pressure with altitude. The lapse layer also cools, ' ...
    'then temperature stays fixed above 11 km; rho=p/(R*T), so fixed-speed q falls.']);
disp('Speed of sound follows sqrt(gamma*R*T), so the same true airspeed has a larger Mach number in colder air.');

%% Lever 2 - reset altitude, then sweep local temperature offset independently
temperatureOffsetSweep_K=[-30 -15 0 15 30];
offsetPressure_kPa=zeros(size(temperatureOffsetSweep_K));
offsetDensity_kgpm3=zeros(size(temperatureOffsetSweep_K));
offsetDynamicPressure_kPa=zeros(size(temperatureOffsetSweep_K));
offsetSoundSpeed_mps=zeros(size(temperatureOffsetSweep_K));
offsetMach=zeros(size(temperatureOffsetSweep_K));
for k=1:numel(temperatureOffsetSweep_K)
    sample=model(5000,temperatureOffsetSweep_K(k),150);
    offsetPressure_kPa(k)=sample.pressure_Pa/1000;
    offsetDensity_kgpm3(k)=sample.density_kgpm3;
    offsetDynamicPressure_kPa(k)=sample.dynamicPressure_Pa/1000;
    offsetSoundSpeed_mps(k)=sample.speedOfSound_mps;
    offsetMach(k)=sample.mach;
end

%% Changed view - warmer air is thinner on one pressure-altitude surface
figure('Name','P03 temperature-offset sweep');
subplot(1,2,1);
yyaxis left;
plot(temperatureOffsetSweep_K,offsetDensity_kgpm3,'o-','LineWidth',1.4);
ylabel('Density (kg/m^3)');
yyaxis right;
plot(temperatureOffsetSweep_K,offsetDynamicPressure_kPa,'s-','LineWidth',1.4);
ylabel('Dynamic pressure q (kPa)');
grid on; xlabel('Temperature offset from standard (K)');
title('Fixed pressure altitude and true airspeed');
subplot(1,2,2);
yyaxis left;
plot(temperatureOffsetSweep_K,offsetSoundSpeed_mps,'o-','LineWidth',1.4);
ylabel('Speed of sound (m/s)');
yyaxis right;
plot(temperatureOffsetSweep_K,offsetMach,'s-','LineWidth',1.4);
ylabel('Mach number (-)');
grid on; xlabel('Temperature offset from standard (K)');
title('Warm air raises a and lowers Mach');

%% Read and explain - mechanism for lever 2
disp(['Mechanism: pressure is fixed by pressure altitude in this local-offset model. ' ...
    'At fixed p, warming lowers rho=p/(R*T), raises a=sqrt(gamma*R*T), and therefore lowers q and Mach.']);
fprintf('Pressure variation across the temperature-offset sweep: %.3g Pa\n', ...
    1000*(max(offsetPressure_kPa)-min(offsetPressure_kPa)));

%% Broken case - freeze density at sea level while climbing to 11 km
failureCase=model(11000,0,150);
seaLevelCase=model(0,0,150);
correctDynamicPressure_kPa=failureCase.dynamicPressure_Pa/1000;
brokenDynamicPressure_kPa=0.5*seaLevelCase.density_kgpm3* ...
    failureCase.trueAirspeed_mps^2/1000;
overpredictionPercent=100*(brokenDynamicPressure_kPa/ ...
    correctDynamicPressure_kPa-1);

figure('Name','P03 broken constant-density assumption');
bar([correctDynamicPressure_kPa brokenDynamicPressure_kPa]); grid on;
set(gca,'XTickLabel',{'local density','broken sea-level density'});
ylabel('Dynamic pressure q at 11 km (kPa)');
title('Same true airspeed, wrong density, wrong aerodynamic load scale');
fprintf(['Broken symptom: sea-level density at 11 km overpredicts q by %.1f%% ' ...
    '(%.2f instead of %.2f kPa).\n'],overpredictionPercent, ...
    brokenDynamicPressure_kPa,correctDynamicPressure_kPa);
assert(brokenDynamicPressure_kPa/correctDynamicPressure_kPa>3.3, ...
    'The deliberately broken constant-density case should strongly overpredict q.');

%% Check and teach back
clear run_checks;
run_checks;
disp(['Teach back in two sentences: connect altitude and temperature to p, T, rho, and a; ' ...
    'then explain how a bad density assumption corrupts q even when true airspeed is correct.']);
