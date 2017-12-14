%============================== swingRight.m =============================
%
%   Calculates the trajectory of the Biped's right leg
%   Called the "SWING (right)" phase
%
%   Run saveSwingRight.m immediately after this if you want to save
%   trajectory data. The script is separate to prevent automated
%   overwriting of trajectory data.
%
%============================== swingRight.m =============================
%% Setup

clear; clc; close all; restoredefaultpath
global nlp

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

my_biped.set_stance('LEFT_FOOT')

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
hl = hl_swingRight;   % trajectory end time
% ============================================



% Trajectory Splicing
% ===================
% joint_vel and joint_val are used to set the initial velocity and joint
% angles, respectively, of sequenctial trajectory scripts. This makes sure
% joint motion is continuous
joint_vel = zeros(3, 2);  % start at rest
% joint_val = [0.115825619353147, 0.011198622603887; 
%             0.050635663291916, -0.153758991427362;
%             -0.065189956061231, -0.065189956061231]; % best init config as of rn

% Optragen Calculation Config
% ===========================
% equation for Optragen cost calxs:
cost_string = 'a1Rd^2+a2Rd^2+a3Rd^2+a1Ld^2+a2Ld^2+a3Ld^2';
ninterv = 2;  % number of trajectory waypoints
smooth = 2;   % smoothness of curve
ord = 3;      % polynomial order
eps = 0.0001;
probName = 'Prob_swingRight';


%% Swing Right

% Lift Right Foot
% ===============
x0 = init_foot_x;
xf = mid_foot_x; 

footUpInit  % Raise foot

% Saving trajectories / intermediate biped state
alpha_traj_swingR = [A1L; A2L; A3L; A1R; A2R; A3R];
alpha_vel_swingR = [A1Ld; A2Ld; A3Ld; A1Rd; A2Rd; A3Rd];
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];


% Lower Right Foot
% ================
xf = end_foot_x;  % don't go all the way to the end of the foot
                  % for mech stability and also Optragen not going wack

footDown  % Lower foot

% Saving SwingRight Trajectory
alpha_traj_swingR = [alpha_traj_swingR, [A1L; A2L; A3L; A1R; A2R; A3R]];
alpha_vel_swingR = [alpha_vel_swingR, [A1Ld; A2Ld; A3Ld; A1Rd; A2Rd; A3Rd]];

% Save end state incase it's needed as an intermediate in the future
joint_val = [A1L(end), A1R(end); A2L(end), A2R(end); A3L(end), A3R(end)];
joint_vel = [A1Ld(end), A1Rd(end); A2Ld(end), A2Rd(end);
             A3Ld(end), A3Rd(end)];

%% Relevant Data (Initial Conditions/Velocity)

init_joints_swingR = reshape(alpha_traj_swingR(:, 1), [3, 2])
my_biped.set_alpha(init_joints_swingR);
init_com_swingR = my_biped.com

%% Animation

my_biped.animateTrajectory(linspace(0, 2*hl_swingRight, 2*100), alpha_traj_swingR);
