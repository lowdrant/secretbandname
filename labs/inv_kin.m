%================================ inv_kin.m ===============================
%
%  Brief demo of Biped() reverse rate inverse kinematics
%
%  Note: This script only runs for the Left Foot.
%        To make it calculate Right Foot, change function IO to match with
%        Right Foot locations. Refer to the README and function header 
%        comments for how Right Foot information is stored.
%
%       The code snippet suspects are: 
%           my_biped.set_alpha      INPUT
%           my_biped.fwd_kinematics OUTPUT
%           a_traj_prime (my_biped.animateTrajectory INPUT)
%
%================================ inv_kin.m ===============================

%% Instantiating Biped

addpath('../');
clear; clc; close all
my_biped = Biped();

my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];  % mm 
my_mtr_dims = [32; 50];

% torso, L0, L1, L2, L3, L4
my_lnk_m = [0, 53.5, 53.5, 53.5, 0, 0];
my_lnk_ctrds = [0,  0,  0,   32, 0, 0;
                0, -25, -25, 0, 0, 0];

my_biped.set_geometry(my_link_lens);

%% Trajectory

a_time = linspace(0, 10, 200);

% Start with desired joint trajectory
% so that calculated joint trajectory can be confirmed
r  = sum(my_link_lens(2:3)) - 5;           % radius from hip joint
a1 = pi/4 * sin(a_time);                   % hip angle
a2 = pi/4 * sin(a_time) - pi/4;             % knee angle
a3 = zeros(size(a1));             % ankle angle
x_des  = r * sin(a1);                      % x distance & coord from torso
y_dist = r * cos(a1) + my_link_lens(1)/2;  % y distance from torso
y_des  = -y_dist;                          % y coordinate from torso

% Additional joint-specific trajectory calculation
% for looking at specific joint behaviors and not just looking at 
% a circular "kick" path
% x_des = my_link_lens(2) .* sin(a1) + my_link_lens(3) .* sin(a1 + a2);
% y_des = -(my_link_lens(2) .* cos(a1) + my_link_lens(3) .* cos(a1 + a2) ...
%         + my_link_lens(1)/2);
% a_des = a1 + a2 + a3;

% FOOT FRAME trajectory (desired)
a_traj = [x_des; y_des; a3];
a_ref_frame = 'TORSO';
a_ee_frame = 'LEFT_FOOT';

% JOINT Trajectory (calculated)
joint_traj = my_biped.rr_inv_kin(a_traj, a_time, a_ref_frame, a_ee_frame);

% Biped plotters take matrix of data for both legs.
% Set other leg to 0 because it doesn't matter to the leg we want
a_traj_prime = [joint_traj; zeros(size(joint_traj))];

%% Deliverable 3A - Joint Angles over Time

figure(1); clf
hold on, xlabel 'time (s)', ylabel 'angle (rad)', title 'a.'
plot(a_time, joint_traj(1, :), 'r')
plot(a_time, joint_traj(2, :), 'g')
plot(a_time, joint_traj(3, :), 'b')
legend('Hip', 'Knee', 'Ankle')
pause(.1)   

%% Deliverable 3B - Parametric Position with 3 Feet Coordinates

figure(2); clf
hold on, xlabel 'position (mm)', ylabel 'position (mm)', title 'b.'
plot(x_des, y_des, 'r')

% Extracting 3 random positions
for ii = randi(size(joint_traj, 2), 1, 3)
    my_biped.set_alpha([joint_traj(:, ii), zeros(3, 1)]);
    [~, LF, ~] = my_biped.fwd_kinematics(a_ref_frame);
    
    LF.plot('LEFT\_FOOT')
end

axis equal

%% Deliverable 3C - Parametric Intended and Calculated Trajectories

% Extracting calculated foot coordinates
posprime = zeros([2, size(joint_traj, 2)]);
for ii = 1:size(joint_traj, 2)
    my_biped.set_alpha([joint_traj(:, ii), zeros(3, 1)]);
    [~, LF, ~] = my_biped.fwd_kinematics(a_ref_frame);
    
    posprime(:, ii) = LF.getTranslation();
end

% Plotting
figure(3); clf
hold on, xlabel 'position (mm)', ylabel 'position (mm)', title 'c.'
plot(posprime(1, :), posprime(2, :), 'b')  % plot first for readability
                                           % y_calc typ. > y_des
plot(x_des, y_des, 'r')
legend('calculated', 'desired')

axis equal
pause(0.1)  % give all figures time to plot

%% Deliverable 4 (Animation)

% Add zero-movement of other leg to trajectory for animate function
% only care about one leg

figure(4), clf
my_biped.animateTrajectory(a_time, a_traj_prime);
