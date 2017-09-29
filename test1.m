%
%
%
%% Setup
clear; clc; close all
dymxpath = ['/home/marion/Documents/GT2017F/ECE4560/project/' ...
            'dynamixel-master/code/matlab/Dynamixel_IO/test_cases'];
        
%% Loading Library

dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(0 , 1);   % connect to port 0, at 1 MBaud

%% Calculating Trajectory

A = 60;              % Degree Amplitude
T = 2;               % Period of Motion
w = 2*pi / T;        % Angular Frequency

stepsize = 0.01;     % stepsize for motion
t = 0:stepsize:5*T;  % sampling motion

th  = sin(w*t);  % Servo Position
dth = diff(th) ./ stepsize;  % Servo Angular Velocity

%% Plotting Trajectories (Sanity Check)

plot(t, th, t(1:end-1), dth)
legend('dest\_pos', 'dest\_speed')

%% Communicating with Skynet

motor_ids = 18;
for ind = 1:length(t)-1
    t1 = tic;  % starting time
    
    dest_pos = th(ind);  % extracting
    dest_speed = dth(ind);
    dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), ...
                               dest_pos, dest_speed);
                           
    t2 = toc;  % ending time
    pause(stepsize - (t2 - t1))
end
