% =======================================================================
%   OCP2NLP
%   Copyright (c) 2005 by
%   Raktim Bhattacharya, (raktim@aero.tamu.edu)
%   Department of Aerospace Engineering
%   Texas A&M University.
%   All right reserved.
% =======================================================================

%% Setup
clear; clc; close all
global nlp;

% Optragen Stuff
addpath('../../');
addpath('../Optragen');  % for optragen package
addpath('../Optragen/src');  % for src files
SNOPTPATH = '../snopt';  % for snopt mex shenanigans
addpath([ SNOPTPATH ]);
addpath([ SNOPTPATH '/matlab/matlab/' ]);  % more snopt mex shananiganes
addpath([ SNOPTPATH '/matlab/mex/' ]);
addpath('../optragen_generated_files')

% Biped Stuff
addpath('../')
my_link_lens = [85.725, 92.075, 76.2, 15.875, 38.1];  % mm 
my_mtr_dims = [32; 50];
my_biped = Biped();
my_biped.set_geometry(my_link_lens);
my_biped.set_stance('TORSO');

%% Trajectory Definition

ninterv = 2;
smooth = 2;
ord = 3;
hl = 1.0;
eps = 0.0001;

l_0 = my_link_lens(1);
l_1 = my_link_lens(2);  % from hip
l_2 = my_link_lens(3);  % from knee
l_3 = my_link_lens(4);  % heel
l_4 = my_link_lens(5);  % toe

% start both on ground, end up like stork
x0 = 0;
y0 = -l_1 - l_2;
xf = -y0;
yf = 0;

%% Trajectories and Constraints

% Generate symbolic representation of end-effector pose
% =====================================================
syms a1 a2 a3  % joint angles
g0_1 = [ cos(a1) -sin(a1) 0; ...
         sin(a1) cos(a1)  0; ...
         0 0 1 ];  % hip frame
g1_2 = [ cos(a2) -sin(a2) 0; ...
         sin(a2) cos(a2)  -l_1; ...
         0 0 1 ];  % knee frame
g2_3 = [ cos(a3) -sin(a3) 0; ...
         sin(a3) cos(a3)  -l_2; ...
         0 0 1 ];  % ankle frame
g3_4 = [1 0 -l_3;
        0 1  0;
        0 0  1];  % heel frame
g3_5 = [1 0 l_4;
        0 1 0;
        0 0 1];  % toe frame

% foot position symbolic functions
g_ee = g0_1 * g1_2 * g2_3;              % spatial to end-effector
g_ee_x_str = char(vpa(g_ee(1, 3), 9));  % end-effector x- spatial position
g_ee_y_str = char(vpa(g_ee(2, 3), 9));  % end-effector y- spatial position

% heel and toe position symbolic functions
g_h = g_ee * g3_4;  % spatial to heel
g_h_y_str = char(vpa(g_h(2, 3), 9));
g_t  = g_ee * g3_5;  % spatial to toe
g_t_y_str = char(vpa(g_t(2, 3), 9));

% joint angle symbolic functions
a1_str = char(vpa(a1, 9));
a2_str = char(vpa(a2, 9));
a3_str = char(vpa(a3, 9));

% Trajectories
% ============
a1 = traj('a1', ninterv, smooth, ord); % Arguments are ninterv, smoothness, order
a2 = traj('a2', ninterv, smooth, ord);
a3 = traj('a3', ninterv, smooth, ord);
% Create derivatives of trajectory variables
a1d = deriv(a1, 'a1');
a2d = deriv(a2, 'a2');
a3d = deriv(a3, 'a3');
ParamList = [];
xVars = {'a1'; 'a2'; 'a3'; 'a1d'; 'a2d'; 'a3d'};

% Define constraints
% ==================
% position constraints
% effectively: "must start here and end there"
pos_constr = constraint(x0, g_ee_x_str, x0, 'initial', xVars) ...
             + constraint(y0, g_ee_y_str, y0, 'initial', xVars) ...
             + constraint(xf, g_ee_x_str, xf, 'final', xVars) ...
             + constraint(yf, g_ee_y_str, yf, 'final', xVars);
% velocity constraints
% "must start at rest"
% %vel_constr = constraint(0, 'a1d', 0, 'initial', xVars) ...
% %               + constraint(0, 'a2d', 0, 'initial', xVars) ...
% %               + constraint(0, 'a3d', 0, 'initial', xVars);

% orientation constraints
% "toe and heel cannot phase through ground"
% also joint movement restriction
ort_constr = constraint(-l_1-l_2-eps, g_h_y_str, Inf, 'trajectory', xVars) ...
            + constraint(-l_1-l_2-eps, g_t_y_str, Inf, 'trajectory', xVars) ...
             + constraint(-pi/2, a1_str, pi/2, 'trajectory', xVars) ...
             + constraint(-pi/2, a2_str, pi/2, 'trajectory', xVars) ...
             + constraint(-pi/2, a3_str, pi/2, 'trajectory', xVars);
Constr = pos_constr + ort_constr;
% Define Cost Function
% ====================
Cost = cost('a1d^2+a2d^2+a3d^2','trajectory'); % Minimise energy


%% Computation

% Collocation Points, using Gaussian Quadrature formula
% I HAVE NO IDEA WHAT THIS DOES
% PRETTY PRETTY PLEASE DON'T BREAK IT
% =====================================================
breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';

HL = [0 colpnts hl];
HL = linspace(0,hl,20);

% Problem Files:
% ===================================================
pathName = './optragen_generated_files';  % Save it all in the current directory.
probName = 'Lab4';

% List of trajectories used in the problem
% ========================================
TrajList = traj.trajList(a1, a1d, a2, a2d, a3, a3d);

nlp = ocp2nlp(TrajList, Cost, Constr, HL, ParamList,pathName,probName);
snset('Minimize');

xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);

Time = linspace(0,1,100);
a1_val = linspace(-pi, pi, 100);  % hip restr
a2_val = linspace(-pi, pi, 100);       % knee restr
a3_val = linspace(-pi, pi, 100);
a1_sp = createGuess(a1,Time,a1_val);
a2_sp = createGuess(a2,Time,a2_val);
a3_sp = createGuess(a3,Time,a3_val);

%init = my_biped.ft_inv_kin([x0; y0; 0]);  % trying to force the guess

init = [a1_sp.coefs a2_sp.coefs a3_sp.coefs]'% + 0.001*rand(nlp.nIC,1);
%init = zeros(nlp.nIC,1);

ghSnopt = snoptFunction(nlp);
tic;
[x, F, inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], ghSnopt);
toc;
F(1);

sp = getTrajSplines(nlp, x);
a1SP = sp{1};
a2SP = sp{2};
a3SP = sp{3};

refinedTimeGrid = linspace(min(HL),max(HL),100);

A1 = fnval(a1SP, refinedTimeGrid);
A1d = fnval(fnder(a1SP), refinedTimeGrid);

A2 = fnval(a2SP, refinedTimeGrid);
A2d = fnval(fnder(a2SP), refinedTimeGrid);

A3 = fnval(a3SP, refinedTimeGrid);
A3d = fnval(fnder(a3SP), refinedTimeGrid);

%% Initial Angles

% a_0 = my_biped.ft_inv_kin([x0; y0; A3(1)]);  % solving for init joint angles
% A1 = [a_0(1), A1];
% A2 = [a_0(2), A2];
% A3 = [a_0(3), A3];

%% Deliverable 1b Animation

a_traj = [A1; A2; A3; zeros(3, length(A1))];
figure(1) clf
my_biped.animateTrajectory(linspace(0, 1, length(A1)), a_traj);
plot(x0, y0-l_0/2, 'd', 'LineWidth', 2)
plot(xf, yf-l_0/2, 'd', 'LineWidth', 2)

%% Deliverable 1a Plotting

figure(2); clf
hold on, xlabel 'Time', ylabel 'Angle (radians)'
plot(A1), plot(A2), plot(A3)
legend('\alpha_1', '\alpha_2', '\alpha_3')
