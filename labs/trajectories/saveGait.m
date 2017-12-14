%============================ saveGait.m ===========================
%
%   Script for saving full gait trajectory data
%
%   loads swingRight and doubleRL trajectories, mirrors them to create the
%   corresponding Left foot trajectories, and then stiches them together as
%   one big file
%
%============================ saveDoubleRL.m ===========================

clc; clear; close all; restoredefaultpath
load swingRightData.mat
load doubleRLData.mat

% Trajectories
alpha_gait_right = [alpha_traj_swingR, alpha_traj_doubleRL];  % right mvmt traj
alpha_gait_left  = alpha_gait_right([4 5 6 1 2 3], :);  % mirror legs for left
alpha_gait       = [alpha_gait_right, alpha_gait_left];  % stitch together

% Velocity
alpha_vel_right = [alpha_vel_swingR, alpha_vel_doubleRL];  % right vel traj
alpha_vel_left  = alpha_vel_right([4 5 6 1 2 3], :);  % mirror legs for left
alpha_vel       = [alpha_vel_right, alpha_vel_left];  % stitch together

% Time
% every trajectory has 100 time points
% swing: takes 2 time lengths per swing
%        => 2*hl for one leg, 4*hl for both
% double: takes 1 time length per double
%        => hl for one double, 2*hl for both
time_gait = linspace(0, 4*hl_swingRight + 2*hl_doubleRL, 600);

% Saving
save('gaitData.mat', 'alpha_gait', 'alpha_vel', 'time_gait');

fprintf('\nSingle gait trajectory saved!\n\n')
 