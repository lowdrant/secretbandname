%============================= comShift.m =============================
%
%   Script for shifting the COM of the Biped with both feet still on the
%   ground
%
%============================= comShift.m =============================
%% Setup

% Biped and Trajectory States:
global nlp  % Optragen thing

% Frame Generation:
% setOptragen saves all the equation strings, so no need
% to double save them here
setOptragen  % Optragen Setup Script


%% Constraints and Costs

% Moving foot position
% ===================
% force foot to stay stationary
pos_constr = constraint(0, MF_y_str, 0, 'trajectory', xVars) ...
             + constraint(xf, MF_x_str, xf, 'trajectory', xVars);

%
% Joint Constraints are handled in calcOptragen.m
% 

% COM constraints
% ===============
%
% COM Stability Constraint is handled in calcOptragen.m
%
com_constr = constraint(COMxf, COM_x_str, COMxf, 'final', xVars);
             
% Velocity constraints
% ====================
% start with vel = 0
% (vf != 0 because this is only part of a foot movement)
vel_constr = ....
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

Constr = pos_constr + com_constr + vel_constr + init_constr;

fprintf('Computing comShift trajectory...\n\n')

calcOptragen  % Calculation Script

fprintf('comShift trajectory generation complete!\n\n')

% Getting out joint balues
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
