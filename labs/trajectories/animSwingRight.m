%============================ animSwingRight.m ===========================
%
%   Animates Swing (Right) trajectory
%
%============================ animSwingRight.m ===========================

% Setup
clear; clc; close all; restoredefaultpath
addpath('../../')
load swingRightData.mat

% Biped
% =====
my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];
my_lnk_m     = [0, 53.5, 53.5, 53.5, 0, 0];
my_lnk_ctrds = [0,  0,  0,   32, 0, 0; 0, -25, -25, 0, 0, 0];
my_biped = Biped();
my_biped.set_com(my_link_lens, my_lnk_m, my_lnk_ctrds);

% Animation
% =========
my_biped.set_stance('LEFT_FOOT')
my_biped.animateTrajectory(linspace(0, 2*hl_swingRight, 2*100), ...
                           alpha_traj_swingR);