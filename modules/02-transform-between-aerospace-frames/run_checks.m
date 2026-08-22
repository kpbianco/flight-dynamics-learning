function run_checks
%RUN_CHECKS Independent analytic checks for the P02 frame transforms.
clear model;
tol=1e-10;

%% Determinism and fixed resource shape
first=model(70,6,0,0,9,30);
second=model(70,6,0,0,9,30);
assert(isequaln(first,second),'Identical inputs must produce identical outputs.');
assert(isequal(size(first.C_wind_to_body),[3 3]) && ...
    isequal(size(first.C_body_to_ned),[3 3]), ...
    'Frame transforms must remain fixed-size 3-by-3 matrices.');
assert(isequal(size(first.velocityWind_mps),[3 1]) && ...
    isequal(size(first.velocityBody_mps),[3 1]) && ...
    isequal(size(first.velocityNed_mps),[3 1]), ...
    'Velocity outputs must remain fixed-size three-component columns.');

%% Independent limiting cases and sign conventions
identityCase=model(70,0,0,0,0,0);
assert(norm(identityCase.C_wind_to_body-eye(3),'fro')<tol && ...
    norm(identityCase.C_body_to_ned-eye(3),'fro')<tol, ...
    'All-zero angles must produce identity transforms.');
assert(norm(identityCase.velocityNed_mps-[70;0;0])<tol, ...
    'With all angles zero, body-forward must point North.');

eastCase=model(70,0,0,0,0,90);
assert(norm(eastCase.velocityNed_mps-[0;70;0])<tol, ...
    'Positive 90 deg yaw must send body-forward velocity East.');

alphaCase=model(70,10,0,0,0,0);
assert(alphaCase.velocityBody_mps(3)>0 && abs(alphaCase.flightPathDeg+10)<tol, ...
    'Positive angle of attack must produce body-down w and a negative flight path here.');

pitchCase=model(70,0,0,0,10,0);
assert(pitchCase.velocityNed_mps(3)<0 && abs(pitchCase.flightPathDeg-10)<tol, ...
    'Positive pitch must send a forward velocity toward negative Down.');

rollCase=model(70,0,30,90,0,0);
expectedRollNed_mps=70*[cosd(30);0;sind(30)];
assert(norm(rollCase.velocityNed_mps-expectedRollNed_mps)<tol, ...
    'Positive 90 deg roll must map positive body-right velocity toward positive Down.');

%% Proper-rotation, composition, norm, and round-trip invariants
general=model(83,7,-4,18,-11,123);
assert(norm(general.C_wind_to_ned- ...
    general.C_body_to_ned*general.C_wind_to_body,'fro')<tol, ...
    'The composed wind-to-NED matrix must equal the two stated transforms.');
assert(general.orthogonalityError<tol, ...
    'Both direction cosine matrices must be orthogonal.');
assert(general.determinantError<tol && ...
    abs(det(general.C_wind_to_ned)-1)<tol, ...
    'Every frame transform must be a proper rotation with determinant +1.');
assert(general.roundTripError_mps<tol && general.normError_mps<tol, ...
    'A forward/inverse round trip and the vector norm must be preserved.');

%% Sweep 1 regression - yaw changes track but not climb angle or speed
yawSweepDeg=[-45 0 45];
yawTrackDeg=zeros(size(yawSweepDeg));
yawFlightPathDeg=zeros(size(yawSweepDeg));
yawSpeed_mps=zeros(size(yawSweepDeg));
for k=1:numel(yawSweepDeg)
    sample=model(70,6,0,0,9,yawSweepDeg(k));
    yawTrackDeg(k)=sample.trackDeg;
    yawFlightPathDeg(k)=sample.flightPathDeg;
    yawSpeed_mps(k)=norm(sample.velocityNed_mps);
end
assert(max(abs(yawTrackDeg-yawSweepDeg))<tol, ...
    'With zero roll and sideslip, air-relative track must follow yaw.');
assert(max(abs(yawFlightPathDeg-3))<tol && max(abs(yawSpeed_mps-70))<tol, ...
    'The yaw sweep must preserve flight-path angle and speed.');

%% Sweep 2 regression - sideslip changes body-right velocity monotonically
betaSweepDeg=[-15 0 15];
lateral_mps=zeros(size(betaSweepDeg));
betaTrackDeg=zeros(size(betaSweepDeg));
for k=1:numel(betaSweepDeg)
    sample=model(70,0,betaSweepDeg(k),0,0,0);
    lateral_mps(k)=sample.velocityBody_mps(2);
    betaTrackDeg(k)=sample.trackDeg;
end
assert(all(diff(lateral_mps)>0) && lateral_mps(1)<0 && lateral_mps(3)>0, ...
    'Body lateral velocity must increase and change sign with sideslip.');
assert(max(abs(betaTrackDeg-betaSweepDeg))<tol, ...
    'At identity attitude and zero alpha, air-relative track must equal sideslip.');

%% Broken-case regression - transpose misuse passes a norm-only check
brokenNed=eastCase.C_body_to_ned.'*eastCase.velocityBody_mps;
assert(norm(brokenNed-[0;-70;0])<tol, ...
    'The reversed transform must expose the expected west-pointing symptom.');
assert(abs(norm(brokenNed)-eastCase.speed_mps)<tol && ...
    norm(brokenNed-eastCase.velocityNed_mps)>100, ...
    'The broken result must preserve speed while producing a large direction error.');

%% Malformed and out-of-chart inputs fail clearly
expectFailure(@() model(0,0,0,0,0,0),'','zero speed');
expectFailure(@() model(-1,0,0,0,0,0),'','negative speed');
expectFailure(@() model([70 80],0,0,0,0,0),'','nonscalar speed');
expectFailure(@() model(70,0,90,0,0,0), ...
    'P02:model:SideslipRange','sideslip chart boundary');
expectFailure(@() model(70,0,0,0,90,0), ...
    'P02:model:PitchSingularity','pitch chart boundary');
expectFailure(@() model(70,0,0,0,0,NaN),'','nonfinite yaw');
afterFailure=model(70,6,0,0,9,30);
assert(isequaln(afterFailure,first), ...
    'A rejected input must not alter the next deterministic calculation.');

disp('P02 checks passed: frame direction, limits, sweeps, malformed inputs, and invariants.');
end

function expectFailure(action,expectedIdentifier,label)
didFail=false;
try
    action();
catch exception
    didFail=true;
    if ~isempty(expectedIdentifier)
        assert(strcmp(exception.identifier,expectedIdentifier), ...
            'Expected %s to raise %s, but received %s.', ...
            label,expectedIdentifier,exception.identifier);
    end
end
assert(didFail,'Expected model to reject %s.',label);
end
