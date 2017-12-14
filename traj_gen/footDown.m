%============================= footDown.m =============================
%
%   Script for lowering a Foot
%
%   Takes into account all initial joint angles and joint velocities
%   Sets final velocity to 0
%
%   Putting the foot down will always be an intermediate step,
%   so there is no need for a footDownInit script
%
%============================= footDown.m =============================
%% Setup

% Biped and Trajectory States:
global nlp  % Optragen thing

% Frame Generation:
% setOptragen saves all the equation strings, so no need
% to double save them here
setOptragen  % Optragen Setup Script

%% Constraints and Costs

% Foot trajectory and ending position
% ===================
% End with foot on the ground
pos_constr = constraint(0, MF_y_str, Inf, 'trajectory', xVars) ...  % foot position
             + constraint(xf, MF_x_str, xf, 'final', xVars) ...
             + constraint(0, MF_y_str, 0, 'final', xVars) ...
             + constraint(0, TOR_x_str, l4, 'final', xVars);  % torso position

%
% Joint Constraints are handled in calcOptragen.m
%

%
% COM Stability Constraint is handled in calcOptragen.m
%

% Velocity constraints
% ====================
% v0 = v_from_last_trajectory
% and
% vf = 0
vel_constr = ...  % intermediate joint velocities
constraint(joint_vel(1,2), 'a1Rd', joint_vel(1,2), 'initial', xVars) ...
+constraint(joint_vel(2,2), 'a2Rd', joint_vel(2,2), 'initial', xVars) ...
+constraint(joint_vel(3,2), 'a3Rd', joint_vel(3,2), 'initial', xVars) ...
+constraint(joint_vel(1,1), 'a1Ld', joint_vel(1,1), 'initial', xVars) ...
+constraint(joint_vel(2,1), 'a2Ld', joint_vel(2,1), 'initial', xVars) ...
+constraint(joint_vel(3,1), 'a3Ld', joint_vel(3,1), 'initial', xVars) ...
    + constraint(0, 'a1Rd', 0, 'final', xVars) ...  % VF=0
    + constraint(0, 'a2Rd', 0, 'final', xVars) ...
    + constraint(0, 'a3Rd', 0, 'final', xVars) ...
    + constraint(0, 'a1Ld', 0, 'final', xVars) ...
    + constraint(0, 'a2Ld', 0, 'final', xVars) ...
    + constraint(0, 'a3Ld', 0, 'final', xVars);

% Initial contraints
% ==================
% Make final pos from last script be same as init of this script
init_constr = ...  % intermediate joint position
constraint(joint_val(1,2), 'a1R', joint_val(1,2), 'initial', xVars) ...
+constraint(joint_val(2,2), 'a2R', joint_val(2,2), 'initial', xVars) ...
+constraint(joint_val(3,2), 'a3R', joint_val(3,2), 'initial', xVars) ...
+constraint(joint_val(1,1), 'a1L', joint_val(1,1), 'initial', xVars) ...
+constraint(joint_val(2,1), 'a2L', joint_val(2,1), 'initial', xVars) ...
+constraint(joint_val(3,1), 'a3L', joint_val(3,1), 'initial', xVars);

%% Computation

Constr = pos_constr + vel_constr + init_constr;

fprintf('Computing footDown trajectory...\n\n')

calcOptragen  % Calculation Script

fprintf('footDown trajectory generation complete!\n\n')

%% Extracting Joint Values:

refinedTimeGrid = linspace(min(HL),max(HL),100);
% right foot
A1R = fnval(a1RSP, refinedTimeGrid);
A1Rd = fnval(fnder(a1RSP), refinedTimeGrid);
A2R = fnval(a2RSP, refinedTimeGrid);
A2Rd = fnval(fnder(a2RSP), refinedTimeGrid);
A3R = fnval(a3RSP, refinedTimeGrid);
A3Rd = fnval(fnder(a3RSP), refinedTimeGrid);
% left foot
A1L = fnval(a1LSP, refinedTimeGrid);
A1Ld = fnval(fnder(a1LSP), refinedTimeGrid);
A2L = fnval(a2LSP, refinedTimeGrid);
A2Ld = fnval(fnder(a2LSP), refinedTimeGrid);
A3L = fnval(a3LSP, refinedTimeGrid);
A3Ld = fnval(fnder(a3LSP), refinedTimeGrid);
