%================================ double.m ==============================
%
%   Transitions Biped COM to setup for next Swing motion
%   keeps feet in static contact with ground
%
%   Change stance before calling this method!! Measures COM relative to
%   the foot which WILL be rested upon in the next swing
%
%=============================== double.m ===============================
%% Setup

clear; clc; close all; restoredefaultpath
global nlp
load swingRightData.mat

% Optragen Paths
% ==============
SNOPTPATH = genpath('../../snopt');
addpath('../../', '../../Optragen', '../../Optragen/src', SNOPTPATH, ...
        '../../optragen_generated_files', '../../traj_gen');

% Biped
% =====
my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];
my_lnk_m     = [0, 53.5, 53.5, 53.5, 0, 0];
my_lnk_ctrds = [0,  0,  0,   32, 0, 0; 0, -25, -25, 0, 0, 0];
my_biped = Biped();
my_biped.set_com(my_link_lens, my_lnk_m, my_lnk_ctrds);
my_biped.set_stance('RIGHT_FOOT')

% shortcuts for joint lengths
l0 = my_link_lens(1);
l1 = my_link_lens(2);
l2 = my_link_lens(3);
l4 = my_link_lens(5);
l3 = my_link_lens(4);
l4 = my_link_lens(5);
    

% ============================================
% Use this to change the trajectory parameters
% ========================
% Trajectory Configuration
% ========================
% use this to change the biped stepping behavior
init_foot_x = -2/3 * l4;  % starting foot pos
end_foot_x = 2/3 * l4;    % end-step foot pos
mid_foot_h = 50;          % mid-step foot height
mid_foot_x = 0;           % mid-step foot pos (I chose halfway init/end)
vlim = pi/4;              % mid-step max joint vel (stops overshoot)
hl_swingRight = 3.0;  % trajectory end time for swingRight
hl_doubleRL = 1.0;  % trajectory end time for doubleRL
% ============================================


% Trajectory Splicing
% ===================
% joint_vel and joint_val are used to set the initial velocity and joint
% angles, respectively, of sequenctial trajectory scripts. This makes sure
% joint motion is continuous
joint_vel = zeros(3, 2);  % start at rest
init_joint_val = reshape(alpha_traj_swingR(:, end), [3, 2]);
temp = reshape(alpha_traj_swingR(:, 1), [3, 2]);
end_joint_val = temp(:, [2, 1]);  % end goal is mirror of start of swing right

init_com = init_com_swingR;

% Optragen Calculation Config
% ===========================
% equation for Optragen cost calxs:
cost_string = 'a1Rd^2+a2Rd^2+a3Rd^2+a1Ld^2+a2Ld^2+a3Ld^2';
ninterv  = 2;    % number of trajectory waypoints
smooth   = 2;    % smoothness of curve
ord      = 3;    % polynomial order
hl = hl_doubleRL;   % trajectory end time
eps      = 0.0001;
probName = 'Prob_doubleRL';

setOptragen;

%% Constraints

% Velocity constraints
% ====================
% Match velocities and end at rest
% Make final pos from last script be same as init of this script
% vel_constr = constraint(0, 'a1Rd', vlim, 'initial', xVars) ...  % V0=0
%              + constraint(0, 'a2Rd', vlim, 'initial', xVars) ...
%              + constraint(0, 'a3Rd', vlim, 'initial', xVars) ...
%              + constraint(0, 'a1Ld', vlim, 'initial', xVars) ...
%              + constraint(0, 'a2Ld', vlim, 'initial', xVars) ...
%              + constraint(0, 'a3Ld', vlim, 'initial', xVars) ...
%              + constraint(0, 'a1Rd', 0, 'final', xVars) ...  % VF=0
%              + constraint(0, 'a2Rd', 0, 'final', xVars) ...
%              + constraint(0, 'a3Rd', 0, 'final', xVars) ...
%              + constraint(0, 'a1Ld', 0, 'final', xVars) ...
%              + constraint(0, 'a2Ld', 0, 'final', xVars) ...
%              + constraint(0, 'a3Ld', 0, 'final', xVars);


% Initial constraints
% ===================
% Match start position with last swing end position
init_constr = ...
constraint(init_joint_val(1,2), 'a1R', init_joint_val(1,2), 'initial', xVars) ...
+constraint(init_joint_val(2,2), 'a2R', init_joint_val(2,2), 'initial', xVars) ...
+constraint(init_joint_val(3,2), 'a3R', init_joint_val(3,2), 'initial', xVars) ...
+constraint(init_joint_val(1,1), 'a1L', init_joint_val(1,1), 'initial', xVars) ...
+constraint(init_joint_val(2,1), 'a2L', init_joint_val(2,1), 'initial', xVars) ...
+constraint(init_joint_val(3,1), 'a3L', init_joint_val(3,1), 'initial', xVars);

% Final constraints
% =================
% Make end position a mirror of the start position
fin_constr = ...
constraint(end_joint_val(1,2), 'a1R', end_joint_val(1,2), 'final', xVars) ...
+constraint(end_joint_val(2,2), 'a2R', end_joint_val(2,2), 'final', xVars) ...
+constraint(end_joint_val(3,2), 'a3R', end_joint_val(3,2), 'final', xVars) ...
+constraint(end_joint_val(1,1), 'a1L', end_joint_val(1,1), 'final', xVars) ...
+constraint(end_joint_val(2,1), 'a2L', end_joint_val(2,1), 'final', xVars) ...
+constraint(end_joint_val(3,1), 'a3L', end_joint_val(3,1), 'final', xVars);

% Foot on ground constraint
traj_constr = constraint(0, MF_y_str, 0, 'trajectory', xVars) ...
              + constraint(init_foot_x, MF_x_str, init_foot_x, 'trajectory', xVars);

%% Computation

Constr = traj_constr + init_constr + fin_constr; % + vel_constr;

fprintf('Computing double trajectory...\n\n')

calcOptragen;

fprintf('double trajectory comlete!\n\n')

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

% Saving Trajectories
alpha_traj_doubleRL = [A1L; A2L; A3L; A1R; A2R; A3R];
alpha_vel_doubleRL = [A1Ld; A2Ld; A3Ld; A1Rd; A2Rd; A3Rd];

%% Init COM

init_joints_doubleRL = reshape(alpha_traj_doubleRL(:, 1), [3, 2])
my_biped.set_alpha(init_joints_doubleRL);
init_com_doubleRL = my_biped.com

%% Animation

my_biped.animateTrajectory(linspace(0, hl, 100), alpha_traj_doubleRL);
