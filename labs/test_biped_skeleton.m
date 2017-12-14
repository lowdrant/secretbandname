%========================== test_biped_skeleton.m =========================
%
%  Brief demo of skeleton Biped() class
%
%========================== test_biped_skeleton.m =========================

%% Instantiate instance/object of the Biped() class
addpath('../')
clear; clc; close all
my_biped = Biped();

% Example Biped class method calls - 
%       Note: these don't do anything as the methods are all stubs  
%             waiting to be filled out by you
my_link_lens = [2; 3+5/8; 3+5/8; 1/2; 1+5/16];  % inches
my_biped.set_geometry(my_link_lens);

% Joint configuration #1
my_joint_angles_1 = [pi/20, pi/4;
                     -pi/8, -pi/3;
                     3*pi/40,  0];
my_biped.set_alpha(my_joint_angles_1);

[my_g_t_lf, my_g_t_rf] = my_biped.fk_torso_foot();
my_biped.set_stance('TORSO');
    
my_biped.plotTF();

%% Joint configuration #2
my_joint_angles_2 = [ pi/6,  0;
                     -pi/6, -pi/6
                     0,      pi/6];
my_biped.set_alpha( my_joint_angles_2 );

[my_g_t_lf, my_g_t_rf] = my_biped.fk_torso_foot();
my_biped.set_stance('RIGHT_FOOT');

my_biped.plotTF();

%% Joint configuration #3
my_joint_angles_3 = [pi/3,    pi/4;
                     pi/4,    -pi/8;
                     0,    0];
my_biped.set_alpha( my_joint_angles_3 );

[my_g_t_lf, my_g_t_rf] = my_biped.fk_torso_foot();
    
my_biped.set_stance('LEFT_FOOT');

my_biped.plotTF();

