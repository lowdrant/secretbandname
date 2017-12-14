%============================= footUpInit.m =============================
%
%   Script for lifting a Foot at the start of a walk
%   
%   Be sure to set stance in the script that calls this one!
%
%   Because it is the inital foot lifting:
%   Does not take into account any initial position
%   Sets initial velocity to 0
%   
%
%============================= footUpInit.m =============================
%% Setup

% Biped and Trajectory States:
global nlp  % Optragen thing

% Frame Generation:
% setOptragen saves all the equation strings, so no need
% to double save them here
setOptragen  % Optragen Setup Script


%% Constraints and Costs

% Right foot position
% ===================
% y0 = 0; x0 = x0; xf = xf; 
% y(t) >= 0; yf = y_mid
pos_constr = constraint(x0, MF_x_str, x0, 'initial', xVars) ...
             + constraint(0, MF_y_str, 0, 'initial', xVars) ...
             + constraint(0, MF_y_str, Inf, 'trajectory', xVars) ...
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
% start with vel = 0
% (vf != 0 because this is only part of a foot movement)
vel_constr = constraint(0, 'a1Rd', 0, 'initial', xVars) ...
             + constraint(0, 'a2Rd', 0, 'initial', xVars) ...
             + constraint(0, 'a3Rd', 0, 'initial', xVars) ...
             + constraint(0, 'a1Ld', 0, 'initial', xVars) ...
             + constraint(0, 'a2Ld', 0, 'initial', xVars) ...
             + constraint(0, 'a3Ld', 0, 'initial', xVars);

%% Computation

Constr = pos_constr + vel_constr;

fprintf('Computing footUpInit trajectory...\n\n')

calcOptragen  % Calculation Script

fprintf('footUpInit trajectory generation complete!\n\n')

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
