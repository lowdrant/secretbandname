%============================ setOptragen.m =============================
%
%   Sets Optragen Variables and Parameters
%
%   Intended to reduce code copy-paste for sequential Optragen runs
%
%   MF -> Moving Foot
%   SF -> Stationary Foot
%
%============================ setOptragen.m =============================
global nlp

syms a1L a2L a3L a1R a2R a3R % joint angles
my_biped.set_alpha([a1L, a1R; a2L, a2R; a3L, a3R]);  % symbolic joints
                %^ don't know if these needs to be set everytime,
                % but for starting out, i'll keep it

[g_TOR, g_MF, g_INTERMED] = my_biped.fwd_kinematics();  % symbolic frames
%       ^MOVING FOOT

g_h = SE2([-l3; 0], 0);  % from ankle to heel (h) and to toe (t)
g_t = SE2([l4; 0], 0);


g_SH = g_h;  % left heel and toe frames relative to LF
g_ST = g_t;
g_MH = g_MF * g_h;  % right heel and toe frames relative to LF
g_MT = g_MF * g_t;

% Homogeneous rep for Optragen (symbolic)
m_SH  = g_SH.getM();
m_ST  = g_ST.getM();
m_MH  = g_MH.getM();
m_MT  = g_MT.getM();
m_MF  = g_MF.getM();
m_TOR  = g_TOR.getM();
pos_com = my_biped.com();  % symbolic COM coords

%% Extracting Relevant Equations

% Center of Mass:
COM_x_str = char(vpa(pos_com(1), 9));  % COM x-pos
COM_y_str = char(vpa(pos_com(2), 9));  % COM y-pos

% Stationary Foot:
SH_x_str  = char(vpa(m_SH(1, 3), 9));  % Heel x-pos
SH_y_str  = char(vpa(m_SH(2, 3), 9));  % Heel y-pos
ST_x_str  = char(vpa(m_ST(1, 3), 9));  % Toe x-pos
ST_y_str  = char(vpa(m_ST(2, 3), 9));  % Toe y-pos

% Moving Foot:
MF_x_str  = char(vpa(m_MF(1, 3), 9));  % Foot x-pos
MF_y_str  = char(vpa(m_MF(2, 3), 9));  % Foot y-pos

MH_x_str  = char(vpa(m_MH(1, 3), 9));  % Heel x-pos
MH_y_str  = char(vpa(m_MH(2, 3), 9));  % Heel y-pos
MT_x_str  = char(vpa(m_MT(1, 3), 9));  % Toe x-pos
MT_y_str  = char(vpa(m_MT(2, 3), 9));  % Toe y-pos

% Torso:
% May be redundant...
TOR_x_str = char(vpa(m_TOR(1, 3), 9));  % Torso pos relative to ankle
TOR_y_str = char(vpa(m_TOR(2, 3), 9));  % Torso height relative to foot

% Joint Angles:
a_ankleR_str = 'a1R+a2R+a3R';
a_ankleL_str = 'a1L+a2L+a3L';

%% Creating Trajectories
% TBPH I don't know if I need to redefine these /every/ time,
% but I'm trying to not break the scripts as a I modularize them rn
%
% Note: Trajectory names be the same as the symbols in the constraints. 
%
% Also: Trajectory derivative names must contain the trajectory name as a
% substring (e.g. A1R -> A1RD, not A1DR). This has to do with how Optragen
% identifies the equations.

% Left Foot Trajectories:
a1L = traj('a1L', ninterv, smooth, ord);
a2L = traj('a2L', ninterv, smooth, ord);
a3L = traj('a3L', ninterv, smooth, ord);
a1Ld = deriv(a1L, 'a1L');
a2Ld = deriv(a2L, 'a2L');
a3Ld = deriv(a3L, 'a3L');

% Right Foot Trajectories:
a1R = traj('a1R', ninterv, smooth, ord);
a2R = traj('a2R', ninterv, smooth, ord);
a3R = traj('a3R', ninterv, smooth, ord);
a1Rd = deriv(a1R, 'a1R');
a2Rd = deriv(a2R, 'a2R');
a3Rd = deriv(a3R, 'a3R');

% Other Optragen Inputs:
ParamList = [];
xVars = {'a1L'; 'a2L'; 'a3L'; 'a1Ld'; 'a2Ld'; 'a3Ld';
          'a1R'; 'a2R'; 'a3R'; 'a1Rd'; 'a2Rd'; 'a3Rd'};