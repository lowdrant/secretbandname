%=========================== gait_ph2_gen ================================
%
%   Transitions Biped COM to setup for next Swing motion
%   keeps feet in static contact with ground
%
%   Change stance before calling this method!! Measures COM relative to
%   the foot which WILL be rested upon in the next swing
%
%============================ threeGaits.m ===============================
%% Setup

clear; clc; close all; restoredefaultpath
addpath('../', 'trajectories/')

% extract trajectory data
load gaitData.mat
gait_left  = alpha_gait(:, 1:floor(end/2));
gait_right = alpha_gait(:, floor(end/2)+1:end);
time = time_gait(1:floor(end/2));
% time = linspace(0, 3*hl, 300);

% initialize biped
my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];
my_lnk_m     = [0, 53.5, 53.5, 53.5, 0, 0];
my_lnk_ctrds = [0,  0,  0,   32, 0, 0; 0, -25, -25, 0, 0, 0];

my_biped = Biped();
my_biped.set_com(my_link_lens, my_lnk_m, my_lnk_ctrds);

figure(1), clf  % for animation methods

%% Gait 1

% Initialize animation at left foot
my_biped.gRO = SE2();  % robot starts at world identity
my_biped.set_stance('LEFT_FOOT')
my_biped.animateTrajectory(time, gait_left)

% change to right foot reference`
[~, new_foot, ~] = my_biped.fwd_kinematics();  % get kinematics to rf
my_biped.gRO = my_biped.gRO * inv(new_foot);   % update worldframe

% right foot animation
my_biped.set_stance('RIGHT_FOOT')
my_biped.animateTrajectory(time, gait_right)

%% Gait 2

% Change to left foot
[~, new_foot, ~] = my_biped.fwd_kinematics();  % get kinematics to lf
my_biped.gRO = my_biped.gRO * inv(new_foot);   % update worldframe

% animate left
my_biped.set_stance('LEFT_FOOT')
my_biped.animateTrajectory(time, gait_left)

% shift to right
[~, new_foot, ~] = my_biped.fwd_kinematics();  % get kinematics to rf
my_biped.gRO = my_biped.gRO * inv(new_foot);   % update worldframe

% animate right
my_biped.set_stance('RIGHT_FOOT')
my_biped.animateTrajectory(time, gait_right)

%% Gait 3

% Change to left foot
[~, new_foot, ~] = my_biped.fwd_kinematics();  % get kinematics to lf
my_biped.gRO = my_biped.gRO * inv(new_foot);   % update worldframe

% animate left
my_biped.set_stance('LEFT_FOOT')
my_biped.animateTrajectory(time, gait_left)

% shift to right
[~, new_foot, ~] = my_biped.fwd_kinematics();  % get kinematics to rf
my_biped.gRO = my_biped.gRO * inv(new_foot);   % update worldframe

% animate right
my_biped.set_stance('RIGHT_FOOT')
my_biped.animateTrajectory(time, gait_right)

