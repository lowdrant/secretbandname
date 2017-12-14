%============================ saveSwingRight.m ===========================
%
%   Script for saving swingRight.m trajectory data
%
%   Don't want to save after every swingRight run just in case something
%   goes wrong, so run this script immediately after swingRight.m if you
%   want to save the data.
%
%============================ saveSwingRight.m ===========================

save('swingRightData.mat', 'init_joints_swingR', 'init_com_swingR', ...
     'alpha_vel_swingR', 'alpha_traj_swingR', 'hl_swingRight')

fprintf('\nSwing Right trajectory saved!\n\n')