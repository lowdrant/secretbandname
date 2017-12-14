%============================ saveDoubleRL.m ===========================
%
%   Script for saving doubleRL.m trajectory data
%
%   Don't want to save after every doubleRL run just in case something
%   goes wrong, so run this script immediately after doubleRL.m if you
%   want to save the data.
%
%============================ saveDoubleRL.m ===========================

save('doubleRLData.mat', 'init_joints_doubleRL', 'init_com_doubleRL', ...
     'alpha_vel_doubleRL', 'alpha_traj_doubleRL', 'hl_doubleRL')

fprintf('\nDouble Right-Left trajectory saved!\n\n')