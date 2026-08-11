function run_checks
a=model(70,25,30,0);
b=model(140,25,30,0);
assert(abs(b.radius/a.radius-4)<1e-10,'Turn radius must scale with speed squared.');
assert(abs(b.turnRate/a.turnRate-0.5)<1e-10,'Turn rate must scale inversely with speed.');
straight=model(70,0,10,0);
assert(isinf(straight.radius) && straight.turnRate==0,'Zero bank should be straight flight.');
steep=model(70,60,10,0);
shallow=model(70,20,10,0);
assert(steep.loadFactor>shallow.loadFactor,'Load factor should rise with bank magnitude.');
disp('P01 checks passed.');
end
