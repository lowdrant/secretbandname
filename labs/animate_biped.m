%============================= animate_biped.m ============================
%
%  Brief demo of Biped() animation method
%
%============================= animate_biped.m ============================

%% Instantiating Biped

addpath('../');
clear; clc; close all
my_biped = Biped();

my_link_lens = [2; 3+5/8; 3+5/8; 1/2; 1+5/16];  % inches
my_biped.set_geometry(my_link_lens);

%% Plot

my_biped.set_com(my_link_lens);
my_alpha = [pi/3,  0;
            -pi/3, 0;
            0, 0];
my_biped.set_alpha(my_alpha);
my_biped.plotTF()

%% Animation

a_time = linspace(0, 7);
A = pi/4;  % radian amplitude of joint angles
w = pi/2;  % frequency of joint behavior
p = [0; 0; 0; pi/2; -pi/6; pi/5]; 
% automating joint angle matrix creation
for a = 1:6
    a_joint_traj(a, :) = A * sin(w*a_time - p(a));
end
a_joint_traj([2, 5], :) = a_joint_traj([2, 5], :) - 1.25*A;  % human-like knees

figure(1), clf
stances = {'TORSO', 'LEFT_FOOT', 'RIGHT_FOOT'};
for k = 1:3
    my_biped.set_stance(stances{k})

    anim_start = tic;
    my_biped.animateTrajectory(a_time, a_joint_traj);
    dt = toc(anim_start)
end
