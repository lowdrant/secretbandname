%
%
%
%% Setup

clear; clc; close all; restoredefaultpath
dymxpath = ['/home/marion/Documents/GT2017F/ECE4560/project/' ...
            'dynamixel-master/code/matlab/Dynamixel_IO/test_cases'];
addpath([dymxpath, '/..'])

%% Loading Library

dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(0 , 1);   % connect to port 0, at 1 MBaud

%% Extracting Trajectory

load('trajectories/swingRightData.mat')

% time for trajectory
t = linspace(0, 2*hl_swingRight, size(alpha_traj_swingR, 2));
step_size = diff(t);

% converting to biped motor angles
% motors measure angles differently than the biped
th            = -alpha_traj_swingR;
th([3, 6], :) = th([3, 6], :) - pi/2;

% velocities
dth = alpha_vel_swingR;

% compliance margin for motors
compl_margin = 0 * ones(size(alpha_traj_swingR));

%% Communicating with Skynet

motor_ids = [9 7 2 6 8 3];  % left leg 0,1,2 | right leg 0,1,2

% Go to init pos
pos = th(:, 1);
vel = dth(:, 1) / 100;

dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), ...
                               pos, vel);

% rest of trajectory
temp = input('Press enter to execute trajectory');                
for ind = 2:length(t)
    t1 = tic;  % starting time
    
    dest_pos = th(:, ind);  % extracting
    dest_speed = dth(:, ind);
    dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), ...
                               dest_pos, dest_speed);
                           
    t2 = toc(t1);  % ending time
    tmp = step_size(ind-1) - t2;
    pause(tmp)
end

