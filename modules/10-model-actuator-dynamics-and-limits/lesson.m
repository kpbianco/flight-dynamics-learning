%% P10 - Model Actuator Dynamics and Limits
% Guiding question:
% What inputs, observable effects, and failure modes matter when you model Actuator Dynamics and Limits?

%% Read - command is not delivered control authority
% P09 integrated body force and moment as prescribed inputs. P10 inserts a
% generic control-surface actuator before the pitch-moment input. Its state
% cannot jump: a first-order lag requests motion, a rate limit bounds how
% fast it moves, and a position envelope bounds how far it may travel. On
% this accepted grid the post-update state clip is a defensive guard; the
% position-limited target keeps every complete candidate inside the envelope.
%
%   delta_dot_raw = (delta_command_limited-delta)/tau
%   delta_dot     = clip(delta_dot_raw,+/-rate_limit)
%   delta         = clip(delta,+/-position_limit)
%   M_y           = (80 N*m/deg) delta  (conceptual body-y ledger)
%
% Positive deflection maps to positive pitch moment only by this declared
% teaching convention. It is not an identified elevator sign or gain, and
% P10 does not provide an adapter into P09's scalar pulse-scale API.

%% Observe, move one lever, observe again, and explain
% Run the fixed experiment before opening the controls. First vary tau while
% holding both limits fixed. Reset, then vary rate authority while holding
% tau and the position envelope fixed. The deliberately broken comparison
% omits both enforcement points of that one envelope, so a smooth surface
% trace can still invent impossible moment authority at the conceptual P09
% boundary.
experiment;

%% Open the live lever panel
% Use the reset button between the time-constant and rate-limit controls.
% Inspect requested, feasible, and delivered deflection before moment or the
% broken comparison.
interactive;

%% Complete the lesson
% Run run_module_checks('P10') from the repository root, then give the
% two-sentence teach-back in checks.md before recording learner completion.
