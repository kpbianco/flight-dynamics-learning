%% P12 - Validate Energy and Frame Conventions
% Guiding question:
% What inputs, observable effects, and failure modes matter when you validate Energy and Frame Conventions?

%% Read - close the loop from specific force to work and energy
% P11 used f_b=C_n_to_b(a_n-g_n) as the ideal accelerometer equation. P12
% rearranges the same relation for a fixed-attitude analytic trajectory:
%
%   a_n = C_body_to_ned f_b + g_n
%
% A proper DCM preserves velocity norms and force-velocity dot products:
%
%   0.5 m (v_b dot v_b) = 0.5 m (v_n dot v_n)
%   m f_b dot v_b       = m f_n dot v_n
%
% Navigation axes are North-East-Down, so altitude h=-Down and
% U=m*g*h=-m*g*Down. Mechanical-energy change must equal accumulated
% non-gravity work. The fixed body frame isolates bookkeeping from attitude
% integration, aerodynamics, estimation, and control.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment first. Vary body-x non-gravity specific force
% while heading and the initial state remain fixed. Reset, then vary heading
% while body histories, altitude, power, work, and energy remain fixed. The
% deliberately broken ledger uses positive Down as height and therefore
% invents unexplained energy despite preserving the entire trajectory.
experiment;

%% Open the live lever panel
% Use the exact reset between the specific-force and heading controls. Read
% the signed NED axes before interpreting the work-energy residual.
interactive;

%% Complete the lesson
% Run run_module_checks('P12') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
