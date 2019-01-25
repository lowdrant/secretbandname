%=========================== calcOptragen.m =============================
%
%   Run Optragen for the script that calls it
%
%=========================== calcOptragen.m =============================
global nlp

% Cost Function
Cost = cost(cost_string, 'trajectory');

% General Constraints
% ===================
% Legs must behave like human legs: (-pi,pi] for hip, ankle
%                                   [-pi/2, pi/2] for knee
% Keep foot level to ground => sum(angles) = 0
gen_constr = constraint(-pi/2, 'a1L', pi/2, 'trajectory', xVars) ...  % joint angles
              + constraint(-pi/2, 'a1R', pi/2, 'trajectory', xVars) ...
              + constraint(-pi, 'a2L', 0, 'trajectory', xVars) ...
              + constraint(-pi, 'a2R', 0, 'trajectory', xVars) ...
              + constraint(-pi/2, 'a3L', pi/2, 'trajectory', xVars) ...
              + constraint(-pi/2, 'a3R', pi/2, 'trajectory', xVars) ...
              + constraint(0, a_ankleL_str, 0, 'trajectory', xVars) ...
              + constraint(0, a_ankleR_str, 0, 'trajectory', xVars) ...
              + constraint(-l3*2/3, COM_x_str, l4, 'trajectory', xVars);  % center of mass
Constr = Constr + gen_constr;


% Collocation Points, using Gaussian Quadrature formula
% =====================================================
breaks = linspace(0, hl, ninterv+1);
gauss = [-1 1] * sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1) + breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)) ,:) + gauss'*diff(breaks);
colpnts = temp(:).';

HL = [0 colpnts hl];
HL = linspace(0, hl, 20);

% Problem Files:
% ===================================================
pathName = '../optragen_generated_files';  % rel to genSource.createHeader

% List of trajectories used in the problem
% ========================================
TrajList = traj.trajList(a1R, a1Rd, a2R, a2Rd, a3R, a3Rd, ...
                         a1L, a1Ld, a2L, a2Ld, a3L, a3Ld);

nlp = ocp2nlp(TrajList, Cost, Constr, HL, ParamList, pathName, probName);
snset('Minimize');

xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);

Time = linspace(0, hl);  % time space for guesses
joint_guess = linspace(-pi, pi);  % joint space for guesses
a1R_sp = createGuess(a1R, Time, joint_guess);
a2R_sp = createGuess(a2R, Time, joint_guess);
a3R_sp = createGuess(a3R, Time, joint_guess);
a1L_sp = createGuess(a1L, Time, joint_guess);
a2L_sp = createGuess(a2L, Time, joint_guess);
a3L_sp = createGuess(a3L, Time, joint_guess);

init = [a1R_sp.coefs a2R_sp.coefs a3R_sp.coefs ...
        a1L_sp.coefs a2L_sp.coefs a3L_sp.coefs];% + 0.001*rand(nlp.nIC,1);

ghSnopt = snoptFunction(nlp);
tic;
[x, F, inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], ghSnopt);
toc;
F(1)


sp = getTrajSplines(nlp, x);
a1RSP = sp{1};
a2RSP = sp{2};
a3RSP = sp{3};
a1LSP = sp{4};
a2LSP = sp{5};
a3LSP = sp{6};
