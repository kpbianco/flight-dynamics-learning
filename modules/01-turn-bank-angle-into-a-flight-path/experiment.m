%% P01 - Turn Bank Angle into a Flight Path
close all; clc;
out=model(70,25,30,2);

figure('Name','P01 baseline');
subplot(1,2,1);
plot(out.x,out.y,'LineWidth',1.3); axis equal; grid on;
xlabel('East (m)'); ylabel('North (m)'); title('Coordinated-turn ground track');
subplot(1,2,2);
yyaxis left; plot(out.t,rad2deg(out.heading),'LineWidth',1.2);
ylabel('Heading change (deg)');
yyaxis right; plot(out.t,out.z,'LineWidth',1.2); ylabel('Altitude change (m)');
grid on; xlabel('Time (s)'); title('Heading and climb');

%% Sweep 1 - bank angle
banks=[10 25 45 60];
figure('Name','P01 bank sweep'); hold on; grid on; axis equal;
for i=1:numel(banks)
    s=model(70,banks(i),30,0);
    plot(s.x,s.y,'LineWidth',1.2,'DisplayName', ...
        sprintf('%g deg, n=%.2f',banks(i),s.loadFactor));
end
xlabel('East (m)'); ylabel('North (m)'); title('Bank trades radius for load factor');
legend('Location','best');

%% Sweep 2 - speed
speeds=[40 70 120];
fprintf('Speed sweep at 25 deg bank:\n');
for i=1:numel(speeds)
    s=model(speeds(i),25,30,0);
    fprintf('  %.0f m/s -> radius %.1f m, turn rate %.2f deg/s\n', ...
        speeds(i),s.radius,rad2deg(s.turnRate));
end

%% Broken case - assume bank alone fixes turn radius
referenceTurn=model(70,25,30,0);
wrongRadius=referenceTurn.radius;
fast=model(120,25,30,0);
t=fast.t;
wrongRate=120/wrongRadius;
xWrong=wrongRadius*sin(wrongRate*t);
yWrong=wrongRadius*(1-cos(wrongRate*t));
figure('Name','P01 broken case');
plot(fast.x,fast.y,'LineWidth',1.3,'DisplayName','Correct at 120 m/s'); hold on;
plot(xWrong,yWrong,'--','LineWidth',1.2,'DisplayName','Broken fixed-radius assumption');
axis equal; grid on; xlabel('East (m)'); ylabel('North (m)');
title('Broken: turn radius grows with speed squared'); legend('Location','best');

assert(out.loadFactor>=1,'Level-turn load factor must be at least one.');
