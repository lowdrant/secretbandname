%================================ COMtest.m ===============================
%
%  Brief demo of Biped() com method
%
%================================ COMtest.m ===============================

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

my_biped.set_com(my_link_lens, my_lnk_m, my_lnk_ctrds);

my_alpha = [pi/3,  0;
            -pi/3, 0;
            0, 0];
my_biped.set_alpha(my_alpha);

%% COM

stances = {'TORSO', 'RIGHT_FOOT', 'LEFT_FOOT'};
for k = 1:3
    my_biped.set_stance(stances{k});
    my_biped.plotTF()
end

%% Animation

a_time = linspace(0, 7);
A = pi/4;  % radian amplitude of joint angles
w = 1/pi;  % frequency of joint behavior
p = [-pi/3; pi/3; 0; pi/2; -pi/6; pi/5]; 

% automating joint angle matrix creation
for a = 1:6
    a_joint_traj(a, :) = A * sin(w*a_time - p(a));
end
a_joint_traj([2, 5], :) = a_joint_traj([2, 5], :) - 1.25*A;  % knees

a_joint_traj([1, 2, 4, 5], :) = zeros(size(a_joint_traj([1, 2, 4, 5], :)));

my_biped.set_stance('TORSO');
figure(), clf
anim_start = tic;
my_biped.animateTrajectory(a_time, a_joint_traj);
dt = toc(anim_start)
