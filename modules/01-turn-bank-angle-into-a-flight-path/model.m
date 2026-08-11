function out = model(speed,bankDeg,duration,climbRate)
%MODEL Coordinated point-mass turn with optional climb.
arguments
    speed (1,1) double {mustBePositive} = 70
    bankDeg (1,1) double = 25
    duration (1,1) double {mustBePositive} = 30
    climbRate (1,1) double = 0
end
g=9.80665;
phi=deg2rad(bankDeg);
turnRate=g*tan(phi)/speed;
if abs(turnRate)<1e-10
    radius=inf;
else
    radius=speed/turnRate;
end
t=linspace(0,duration,600);
heading=turnRate*t;
if isinf(radius)
    x=speed*t; y=zeros(size(t));
else
    x=radius*sin(heading);
    y=radius*(1-cos(heading));
end
z=climbRate*t;
loadFactor=1/max(cos(phi),eps);
out=struct('t',t,'x',x,'y',y,'z',z,'heading',heading,'turnRate',turnRate, ...
    'radius',radius,'loadFactor',loadFactor,'speed',speed,'bankDeg',bankDeg);
end
