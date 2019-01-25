%============================== gaitGen.m =============================
%
%   Master script for trajectory generation
%
%============================== gaitGen.m =============================
%% Setup

clear; clc; close all; restoredefaultpath
global nlp

% Optragen Paths
% ==============
SNOPTPATH = genpath('../snopt');
addpath('../', '../Optragen', '../Optragen/src', SNOPTPATH, ...
        '../optragen_generated_files');

% Biped
% =====
my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];
my_lnk_m     = [0, 53.5, 53.5, 53.5, 0, 0];
my_lnk_ctrds = [0,  0,  0,   32, 0, 0; 0, -25, -25, 0, 0, 0];
my_biped = Biped();
my_biped.set_com(my_link_lens, my_lnk_m, my_lnk_ctrds);
l0 = my_link_lens(1);  % important for the other scripts
l1 = my_link_lens(2);  % important for the other scripts
l2 = my_link_lens(3);  % important for the other scripts
l4 = my_link_lens(5);  % important for the other scripts
l3 = my_link_lens(4);  % important for the other scripts
l4 = my_link_lens(5);  % important for the other scripts



% Trajectory Configuration
% use this to change the biped stepping behavior
% ========================
mid_foot_h = 10;        % mid-step foot hight
mid_foot_x = 1/3 * l4;  % mid-step foot pos
end_foot_x = 2/3 * l4;  % end-step foot pos
vlim = pi/4;            % mid-step max joint vel (stops overshoot)



% Trajectory Splicing
% ===================
% joint_vel and joint_val are used to set the initial conditions
% of sequential trajectory scripts. This prevents impulsive acceleration
% or sudden position shifts
joint_vel = zeros(3, 2);  % velocities of each joint
joint_val = zeros(3, 2);  % positions of each joint

% Optragen Calculation Config
% ===========================
% equation for Optragen cost calxs:
cost_string = 'a1Rd^2+a2Rd^2+a3Rd^2+a1Ld^2+a2Ld^2+a3Ld^2';
ninterv = 2;  % number of trajectory waypoints
smooth = 2;   % smoothness of curve
ord = 3;      % polynomial order
hl = 5.0;     % trajectory end time
eps = 0.0001;
probName = 'Lab7';


%% Gait Generation

% Lift Right Foot
% ===============
my_biped.set_stance('LEFT_FOOT')
x0 = -end_foot_x/2;
xf = mid_foot_x; 

footUpInit  % Start the first step

% Saving trajectories / intermediate biped state
alpha_traj_master = [A1L; A2L; A3L; A1R; A2R; A3R];
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];

         
% Lower Right Foot
% ================
xf = end_foot_x;  % don't go all the way to the end of the foot
                  % Because COM passing between feet and also if
                  % too much distance between start and stop, Optragen
                  % slides the foot on the ground

footDown  % Lower foot

alpha_traj_master = [alpha_traj_master, [A1L; A2L; A3L; A1R; A2R; A3R]];
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];

         
% Shift COM
% =========
my_biped.set_stance('RIGHT_FOOT')
xf = -xf;  % Switching reference foot, so the x-position is flipped
COMxf = 4/5 * end_foot_x;

comShift  % Shift center of mass

alpha_traj_master = [alpha_traj_master, [A1L; A2L; A3L; A1R; A2R; A3R]];
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];

         
% Lift Left Foot
% ==============
xf = mid_foot_x;

footUpIntermed  % Lift Foot

% append trajectory
alpha_traj_master = [alpha_traj_master, [A1L; A2L; A3L; A1R; A2R; A3R]];
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];
         
         
% Lower Left Foot
% ===============
xf = end_foot_x;

footDown  % lower foot

% append trajectory
alpha_traj_master = [alpha_traj_master, [A1L; A2L; A3L; A1R; A2R; A3R]];
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];
         
%% Animation/Plotting

n = size(alpha_traj_master, 2) / 100;  % Each optragen run adds 100 points
                                       % Divide by 100 to get the number of
                                       % runs, which tells you how long the
                                       % run took
my_biped.animateTrajectory(linspace(0, n*hl, n*100), alpha_traj_master);

