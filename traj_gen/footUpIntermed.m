%========================= footUpIntermed.m =============================
%
%   Script for lifting a foot during a walk
%
%   Takes into account initial joint angles and velocities
%   Only sets ending position
%
%   Don't forget to change Biped stance!!
%
%========================= footUpIntermed.m =============================
%% Setup

% Biped and Trajectory States:
global nlp  % Optragen thing
my_biped.set_stance('RIGHT_FOOT')

% Frame Generation:
% setOptragen saves all the equation strings, so no need
% to double save them here
setOptragen  % Optragen Setup Script

%% Constraints and Costs

% Right foot position
% ===================
% y0 = 0; x0 = x0; xf = xf; 
% y(t) >= 0; yf = y_mid
pos_constr = constraint(0, MF_y_str, Inf, 'trajectory', xVars) ...
             + constraint(xf, MF_x_str, xf, 'final', xVars) ...
             + constraint(mid_foot_h, MF_y_str, mid_foot_h, 'final', xVars);

%
% Joint Constraints are handled in calcOptragen.m
%

%
% COM Stability Constraint is handled in calcOptragen.m
%
             
% Velocity constraints
% ====================
% v0 = v_from_last_trajectory
% (should be 0)
vel_constr = ....
constraint(joint_vel(1,2), 'a1Rd', joint_vel(1,2), 'initial', xVars) ...
+constraint(joint_vel(2,2), 'a2Rd', joint_vel(2,2), 'initial', xVars) ...
+constraint(joint_vel(3,2), 'a3Rd', joint_vel(3,2), 'initial', xVars) ...
+constraint(joint_vel(1,1), 'a1Ld', joint_vel(1,1), 'initial', xVars) ...
+constraint(joint_vel(2,1), 'a2Ld', joint_vel(2,1), 'initial', xVars) ...
+constraint(joint_vel(3,1), 'a3Ld', joint_vel(3,1), 'initial', xVars) ...
    + constraint(-vlim, 'a1Rd', vlim, 'final', xVars) ...  % VF=0
    + constraint(-vlim, 'a2Rd', vlim, 'final', xVars) ...
    + constraint(-vlim, 'a3Rd', vlim, 'final', xVars) ...
    + constraint(-vlim, 'a1Ld', vlim, 'final', xVars) ...
    + constraint(-vlim, 'a2Ld', vlim, 'final', xVars) ...
    + constraint(-vlim, 'a3Ld', vlim, 'final', xVars);

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

fprintf('Computing footUpIntermed trajectory...\n\n')

calcOptragen  % Calculation Script

fprintf('footUpIntermed trajectory generation complete!\n\n')

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
