%============================ anim_double.m ==============================
%
%   Animates the biped executing the Double (Right-Left) gait pattern
%
%   Time length was 5s IIRC
%
%============================ anim_double.m ==============================
%% Setup

clear; clc; close all; restoredefaultpath
addpath('../', 'trajectories/')
load doubleRLData.mat

my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];
my_lnk_m     = [0, 53.5, 53.5, 53.5, 0, 0];
my_lnk_ctrds = [0,  0,  0,   32, 0, 0; 0, -25, -25, 0, 0, 0];

my_biped = Biped();
my_biped.set_com(my_link_lens, my_lnk_m, my_lnk_ctrds);
my_biped.set_stance('LEFT_FOOT')

%% Animation

figure(1), clf
my_biped.animateTrajectory(linspace(0, 5), alpha_traj_doubleRL)
