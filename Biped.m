%================================ Biped ===============================
%
% @class    Biped
%
% @brief    Model implementation for bipedal robots & supporting utilities 
%
%  
%  Skeleton code with stub'd out functions - to be completed as part of 
%  ECE 4560 weekly course assignments.
%
%  Note: Should utilize SE2 Matlab class
%
%================================ Biped ===============================

%
% @file     Biped.m
%
% @author   Alex H. Chang,   alexander.h.chang@gatech.edu
% @date     2017/09/26 [created]
%
% @note
%   set indent to 2 spaces.
%   set tab to 4 spaces, with conversion.
%
%================================ Biped ===============================

classdef Biped < handle
  
  
  properties
    gRO = [];        % biped world frame
      
    alpha = [];      % current biped joint configuration (angles RADIANS)
    geom = [];       % biped joint lengths (millimeters)
    stance = [];     % ref frame of biped ('TORSO'|'LEFT_FOOT'|'RIGHT_FOOT')
    lnk_m = [];      % mass of each link (NOT MOTORS)
    lnk_ctrd = []    % centroid of each link    
    
    % local frames
    gT0_L = [];  % left leg
    g01_L = [];
    g12_L = [];
    gTF_L = [];
    
    gT0_R = [];  % right leg
    g01_R = [];
    g12_R = [];
    gTF_R = [];
    
    g23 = [];  % front/back of feet
    g24 = [];
  end
  
  %
  %
  %============================ Member Functions ==========================
  %
  %
  methods
    
    %------------------------------ Biped ------------------------------
    %
    %  Constructor for the class. Currently takes no arguments.
    %  If no args, then initialize as identity.
    %
    function obj = Biped()

        obj.gRO = SE2([0; 0], 0);
        obj.stance = 'TORSO';
        
    end
    
    %------------------------- set_geometry ----------------------------
    %
    % set_geometry(a_geom)
    %
    % Sets the link lengths associated with the biped object. The biped
    % legs are symmetrical, so a_geom is a vector. Link length indicies are
    % as follows: TORSO, UPPER_LEG, LOWER_LEG, BACK_FOOT, FRONT_FOOT.
    %
    % Call this method before set_alpha. set_alpha calls update_linkframes,
    % which will error if the geometry is not set beforehand.
    %
    function set_geometry(obj, a_geom)
     %BIPED.SET_GEOMETRY() Set biped link lengths.
     %  BIPED.SET_GEOMETRY(a_geom) Expects length-5 vector of link lengths.
     %
     %  a_geom: [L0; L1; L2; L3; L4]
     %  See README for link diagram.
     
        obj.geom = a_geom;
        
    end
    
    %---------------------------- set_alpha ----------------------------
    %
    % set_alpha(a_alpha)
    %
    % Sets the joint configurations for the biped. Expects a 3x2 matrix of
    % angles in RADIANS. Angle indicies follow link length indicies. Does
    % not set the pose of the biped with respect to the world frame. 
    %
    % Call after set_geometry.
    %
    function set_alpha(obj, a_alpha)
    %BIPED.SET_ALPHA() Set biped joint angles.
    %   BIPED.SET_ALPHA(a_alpha) Expects 3x2 matrix of joint angles in
    %                            RADIANS. Call after SET_GEOMETRY or
    %                            SET_COM.
    %   Rows:     a0; a1; a2
    %   Column 1: Left leg
    %   Column 2: Right leg
        
        obj.alpha = a_alpha;
        obj.update_linkframes();  % update frames when config changes
        
    end
    
    %---------------------------- set_stance ----------------------------
    %
    %  Sets the Biped reference frame for plotting and COM calculations.
    %
    %  >> set_stance(a_stance)
    %
    %  Expected Values: 'TORSO', 'LEFT_FOOT', or 'RIGHT_FOOT'.
    %
    function set_stance(obj, a_stance)
    %BIPED.SET_STANCE(a_stance) Set reference frame of Biped
    %Acceptable Values: 'TORSO', 'LEFT_FOOT', 'RIGHT_FOOT'

        if ~(strcmp(a_stance, 'TORSO') ...  % validate user input
             || strcmp(a_stance, 'LEFT_FOOT') ...
             || strcmp(a_stance, 'RIGHT_FOOT') ...
            )
            error(['Stance can only have the values ''TORSO'',', ...
                   '''LEFT_FOOT'', or ''RIGHT_FOOT''']);
        end
        obj.stance = a_stance;
        
    end

    %----------------------------- set_lnks ----------------------------
    %
    %  Sets link masses and their respective centroid locations. The
    %  centroid locations are relative to that link's frame.
    %
    %  >> set_lnks(a_lnk_m, a_lnk_ctrd)
    %
    %  a_lnk_m is a 1x6 vector of link masses.
    %  a_lnk_ctrd is a 2x6 matrix of link centroid XY coordinates.
    %
    %  Link indicies increase going down the leg: 
    %    0,     1,  2,  3,  4,  5
    %    TORSO, L0, L1, L2, L3, L4. 
    %
    %  Assumes legs are identical.
    %
    function set_lnks(obj, a_lnk_m, a_lnk_ctrd)
    %BIPED.SET_LNKS set link mass properties.
    %   BIPED.SET_LNKS(a_lnk_m, a_lnk_ctrd)
    %       Columns:  TORSO, L0, L1, L2, L3, L4
    %       a_lnk_m:  1x6 vector of link masses
    %       a_ctrd_m: 2x6 matrix of link centroid XY
    %                 coordinates relative to that link's frame

        if ~all(size(a_lnk_m) == [1, 6]) || length(a_lnk_m) == 6
            a_lnk_m = reshape(a_lnk_m, [1, 6]);
        else
            error('a_lnk_m must be a 1x6 vector.');
        end
    
        obj.lnk_m = a_lnk_m;
        obj.lnk_ctrd = a_lnk_ctrd;

    end
    
    %----------------------------- set_com ----------------------------
    %
    %  Sets Biped COM and size data. Combines a_geom and a_lnk_m
    %  internally.
    %
    %  >> set_com(a_geom, a_lnk_m, a_lnk_ctrd)
    %
    %  If COM member data is not set, plotTF will ignore internal COM
    %  calculation calls.
    %
    %  See set_geom and set_lnks for information about the inputs
    %
    function set_com(obj, a_geom, a_lnk_m, a_lnk_ctrd)
    %BIPED.SET_COM(a_geom, a_lnk_m, a_lnk_ctrd)
    %Sets geometric and mass properties of biped all at once
    %
    
        % avoid setting COM data if user errors
        if (nargin ~= 4)
            error('set_com(a_geom, a_lnk_m, a_lnk_ctrd')
        end
        
        obj.geom = a_geom;
        obj.lnk_m = a_lnk_m;
        obj.lnk_ctrd = a_lnk_ctrd;
        
    end
     
    %------------------------------ com --------------------------------
    %
    %  Calculates the XY planar COM of the biped relative to the reference
    %  frame. Treats each link as a point mass and calculates the COM of
    %  the points.
    %
    %  Returns a 2x1 vector of COM coordinates
    %
    %  Can be invoked in two ways:
    %
    %  >> com(a_ref_frame)
    %
    %  OR
    %
    %  >> com()
    %
    %  The no-arg case allows the user to calculate the COM relative to
    %  the previously set stance, stored in obj.stance. 
    %
    %  Otherwise, com sets the stance to a_ref_frame and executes exacly
    %  the executes the same.
    %
    function pos_com = com(obj, a_ref_frame)
    %BIPED.COM(a_ref_frame)  Biped com location relative to a_ref_frame
    %
    %   Returns [x; y] vector of com coordinates
    %
    %   If no args given, a_ref_fr defaults to Biped.stance

        if nargin < 2
            a_ref_frame = obj.stance;
        end
        
        % Orientation & Mass Data
        [fr1, fr2, b_frs] = obj.fwd_kinematics(a_ref_frame);
        
        M =  2*sum(obj.lnk_m) - obj.lnk_m(1);  % total biped mass
                                               % 2*legs + 2*torso - torso
                                               
        ctrds = obj.lnk_ctrd;  % extra centroid var for brevity        
        
        % Reference Frame Transformations
        % locations transformable everywhere
        lnk_com = [b_frs(1, 1) .* ctrds(:, 2), ...  % L0
                   b_frs(1, 2) .* ctrds(:, 2), ...
                   b_frs(2, 1) .* ctrds(:, 3), ...  % L1
                   b_frs(2, 2) .* ctrds(:, 3), ...
                   fr2 .* ctrds(:, 4),  ...         % Foot (L2, L3, L4)
                   fr2 * obj.g23 .* ctrds(:, 5), ...
                   fr2 * obj.g24 .* ctrds(:, 6)];
        % ref-frame-dependent locations
        switch(obj.stance(1))
            case 'T'
                lnk_com = [ctrds(:, 1), ... % torso is ref frame
                           lnk_com, ...
                           fr1 .* ctrds(:, 4), ...  % other foot
                           fr1 * obj.g23 .* ctrds(:, 5), ...
                           fr1 * obj.g24 .* ctrds(:, 6)];
            otherwise
                lnk_com = [fr1 .* ctrds(:, 1), ... % Torso
                           lnk_com, ...
                           ctrds(:, 4), ...  % a foot is the ref frame
                           obj.g23 .* ctrds(:, 5), ...
                           obj.g24 .* ctrds(:, 6)];
        end
        
        % COM Calculation
        pt_m = obj.lnk_m([1, 2, 2, 3, 3, 4, 5, 6, 4, 5, 6;
                          1, 2, 2, 3, 3, 4, 5, 6, 4, 5, 6]) .* lnk_com;
        pos_com = sum(pt_m, 2) / M;
 
    end

    %------------------------- fk_torso_foot ---------------------------
    %
    % Return SE(2) poses representing left foot frame & right foot frame,
    % both relative to the torso frame 
    % (ie. g^t_lf and g^t_rf, respectively)
    %
    function [g_t_lf, g_t_rf] = fk_torso_foot(obj)
    %[g_torso_leftfoot, g_torso_rightfoot] = BIPED.FK_TORSO_FOOT()
        
        g_t_lf = obj.gTF_L;
        g_t_rf = obj.gTF_R;
        
    end
    
    %----------------------------- plotTf ------------------------------
    %
    % plotTF() or plotTF(anim)
    %
    % Plot the torso frame, both foot frames and all links, consistent
    % with the biped's current joint configuration, as well as the center
    % of mass and the Biped's world frame.
    %
    % The anim argument determines whether plotTF is being used in an
    % animation or not. If plotTF is called with no arguments, or anim=0 
    % or 'false', a new figure is created. If anim is anything else,
    % plotTF assumes an animation is ongoing and calls clf() instead of
    % figure().
    %
    % To NOT plot the COM, make lnk_m an empty vector.
    % To NOT plot world frame, make gRO an empty vector.
    %
    function plotTF(obj, anim)
    %BIPED.PLOTTF Plot/Animate biped skeleton at current joint config.
    %   BIPED.PLOTTF() Plots biped skeleton by creating new figure.
    %
    %   BIPED.PLOTTF(anim) Animates biped skeleton if anim == true.
    %    Overwrites figure instead of creating a new one.
        
        % Calculate Link Locations
        % ========================
        [fr1, fr2, biped_frs] = obj.fwd_kinematics(obj.stance);
        % Ensure reference frame string displays properly in plot
        s = obj.stance;
        if s(1) ~= 'T'  % account for '_' in 'LEFT_FOOT' and 'RIGHT_FOOT'
            s = [s(1:strfind(s, '_')-1), '\', s(strfind(s, '_'):end)];
        end
        % Extracting link coordiantes relative to reference frame
        L1_L = biped_frs(1, 1).getTranslation();
        L2_L = biped_frs(2, 1).getTranslation();
        L1_R = biped_frs(1, 2).getTranslation();
        L2_R = biped_frs(2, 2).getTranslation();
        switch obj.stance(1)
            case 'T'  % TORSO
                torso = [0; 0];                 % torso location
                RFOOT = fr1.getTranslation();   % right foot location
                LFOOT = fr2.getTranslation();   % left foor location
                RFB = fr1 * obj.g23 .* [0; 0];  % right foot back
                RFF = fr1 * obj.g24 .* [0; 0];  % right foot front
                LFB = fr2 * obj.g23 .* [0; 0];  % left foot back
                LFF = fr2 * obj.g24 .* [0; 0];  % left foot front
            case 'L'  % LEFT_FOOT
                torso = fr1.getTranslation();
                RFOOT = fr2.getTranslation();
                LFOOT = [0; 0];
                RFB = fr2 * obj.g23 .* [0; 0];
                RFF = fr2 * obj.g24 .* [0; 0];
                LFB = [-obj.geom(4); 0];
                LFF = [obj.geom(5); 0];
            case 'R'  % RIGHT_FOOT
                torso = fr1.getTranslation();
                RFOOT = [0; 0];
                LFOOT = fr2.getTranslation();
                RFB = [-obj.geom(4); 0];
                RFF = [obj.geom(5); 0];
                LFB = fr2 * obj.g23 .* [0; 0];
                LFF = fr2 * obj.g24 .* [0; 0];
        end
        
        
        % Plotting
        % ========
        % Animation flag check
        if nargin < 2 || isequal(anim, 0) || strcmpi(anim, 'false')
            figure()
        else
            hold off  % hold off overwrites the animation figure, whereas
                      % clf overwrites whatever the active figure is
        end
        % Plotting Frames
        SE2([0; 0], 0).plot(s)  % ref frame w/frame name
        hold on
        fr1.plot('fr1')
        fr2.plot('fr2')
        if isa(obj.gRO, 'SE2')
            obj.gRO.plot('gRO')
        end
        % Plotting Links
        plot([torso(1), L1_L(1)], [torso(2), L1_L(2)], 'k-o')
        plot([L1_L(1), L2_L(1), LFOOT(1), LFF(1), LFB(1)], ...
             [L1_L(2), L2_L(2), LFOOT(2), LFF(2), LFB(2)], 'c-o')
        plot([L1_R(1), L2_R(1), RFOOT(1), RFF(1), RFB(1)], ...
             [L1_R(2), L2_R(2), RFOOT(2), RFF(2), RFB(2)], 'r-o')
        % Plotting COM 
        if ~isempty(obj.lnk_m)  % make sure COM data is set before calcs
            pos_com = obj.com(obj.stance);
            plot(pos_com(1), pos_com(2), 'gX', 'MarkerSize', 30)
            text(pos_com(1), pos_com(2), sprintf('(%.1f,%.1f)', pos_com))
        end
        
    end
    
    %------------------------ update_linkframes ------------------------
    %
    %  Updates the frames relating links.
    %
    %  User should generally not have to explicitly call this function, as
    %  set_alpha calls it internally.
    %
    %  Zero-Configuration: Y-Axes of all joints in a vertical line with
    %  the foot parallel to the ground
    %
    function update_linkframes(obj)
    %BIPED.UPDATE_LINKFRAMES Update joint-to-joint transformations.
    %   BIPED.UPDATE_LINKFRAMES() Called internally to update link frames
    %   when joint angles change. User should NOT have to call this.
        
        % left leg
        obj.gT0_L = SE2([0; -obj.geom(1)/2], obj.alpha(1, 1));
        obj.g01_L = SE2([0; -obj.geom(2)], obj.alpha(2, 1));
        obj.g12_L = SE2([0; -obj.geom(3)], obj.alpha(3, 1));

        % right leg
        obj.gT0_R = SE2([0; -obj.geom(1)/2], obj.alpha(1, 2));
        obj.g01_R = SE2([0; -obj.geom(2)], obj.alpha(2, 2));
        obj.g12_R = SE2([0; -obj.geom(3)], obj.alpha(3, 2));
    
        % torso to feet
        obj.gTF_L = obj.gT0_L * obj.g01_L * obj.g12_L;
        obj.gTF_R = obj.gT0_R * obj.g01_R * obj.g12_R;

        % foot branches
        % branches extend along the foot frame x-axis
        obj.g23 = SE2([-obj.geom(4), 0], 0);  % back of foot
        obj.g24 = SE2([obj.geom(5), 0], 0);  % front of foot
                
    end
    
    %-------------------------- fwd_kinematics -------------------------
    %
    %    Computes Biped forward kinematics.
    %
    %
    % >> fwd_kinematics()
    % OR
    % >> fwd_kinematics(a_ref_fr)
    %
    % Computes biped forward kinematics relative to the given a_ref_fr.
    % Expects 'TORSO', 'LEFT_FOOT', or 'RIGHT_FOOT'.
    %
    % Returns the other two frames and a matrix of the intermediate biped
    % frames relative to a_ref_fr.
    %
    % case TORSO
    %     fr1 = right_foot
    %     fr2 = left_foot
    % case LEFT_FOOT
    %     fr1 = torso
    %     fr2 = right_foot
    % case RIGHT_FOOT
    %     fr1 = torso
    %     fr2 = left_foot
    %
    % biped_frs format: [g0L, g0R; 
    %                    g1L, g1R]
    %
    function [fr1, fr2, biped_frs] = fwd_kinematics(obj, a_ref_fr)
    %BIPED.FWD_KINEMATICS Get transformations between major Biped frames
    %[fr1, fr2, biped_frs] = BIPED.FWD_KINEMATICS(a_ref_fr)
    %    BIPED.FWD_KINEMATICS() Returns frames relative to the current value
    %    of Biped.stance.
    %
    %    BIPED.FWD_KINEMATICS(a_ref_fr) Returns the frames relative to
    %    a_ref_fr. Does NOT update Biped.stance. Expected values for
    %    a_ref_fr are: 'TORSO', 'LEFT_FOOT', 'RIGHT_FOOT'. 
    %
    %   ------------------------------
    %   Output for each a_ref_fr value
    %   ------------------------------
    %   TORSO:
    %       fr1 = g^TORSO_RIGHTFOOT
    %       fr2 = g^TORSO_LEFTFOOT
    %   LEFT_FOOT:
    %       fr1 = g^LEFTFOOT_TORSO
    %       fr2 = g^LEFTFOOT_RIGHTFOOT
    %   RIGHT_FOOT:
    %       fr1 = g^RIGHTFOOT_TORSO
    %       fr2 = g^RIGHTFOOT_LEFTFOOT
    %   ------------------------------
    %   Format of biped_frs
    %   ------------------------------
    %   [ glink0Left, glink0Right;
    %     glink1Left, glink1Right ]
        
        if nargin == 2
            prev_stance = obj.stance;
            obj.set_stance(a_ref_fr);
        else
            a_ref_fr = obj.stance;
        end
        
        first_letter = a_ref_fr(1);  % use 1st char to do switch-case
        switch first_letter
            case 'T'  % 'TORSO'
                fr1 = obj.gTF_R;
                fr2 = obj.gTF_L;
                biped_frs = [obj.gT0_L,           obj.gT0_R;
                             obj.gT0_L*obj.g01_L, obj.gT0_R*obj.g01_R];
            case 'L'  % 'LEFT_FOOT'
                fr1 = inv(obj.gTF_L);
                fr2 = fr1 * obj.gTF_R;
                biped_frs(:, 1) = [obj.g12_L.inv() * obj.g01_L.inv();
                                   obj.g12_L.inv()];
                biped_frs(:, 2) = [fr1 * obj.gT0_R;
                                   fr1*obj.gT0_R*obj.g01_R];
            case 'R'  % 'RIGHT_FOOT'
                fr1 = obj.gTF_R.inv();
                fr2 = fr1 * obj.gTF_L;
                biped_frs(:, 1) = [fr1 * obj.gT0_L;
                                   fr1*obj.gT0_L*obj.g01_L];
                biped_frs(:, 2) = [obj.g12_R.inv() * obj.g01_R.inv();
                                   obj.g12_R.inv()];
        end
       
        if nargin == 2
            obj.set_stance(prev_stance);
        end
    end
    
    %------------------------ animateTrajectory ------------------------
    %
    %  Animates biped trajectory over N time instants. 
    %
    %  >> animateTrajectory(a_time, a_joint_traj)
    %
    %  Expects an Nx1 vector of time instants (a_time) and a 6xN matrix of
    %  joint angles (a_joint_traj).
    %
    %  a_joint_traj indexes like so: L0, L1, L2, R0, R1, R2
    %
    function animateTrajectory(obj, a_time, a_joint_traj)
    %BIPED.ANIMATETRAJECTORY(a_time, a_joint_traj)
    %
    %    a_time:       length-N vector of time instants
    %    a_joint_traj: 6xN matrix of joint angles
    %                  L0; L1; L2; R0; R1; R2
    
        % Setup
        pauses = diff(a_time);  % delay b/t frames
        pauses(end+1) = 0;  % prevent trailing delay after last frame
        alpha_traj(1:3, 1, :) = a_joint_traj(1:3, :);  % reshape input traj
        alpha_traj(1:3, 2, :) = a_joint_traj(4:6, :);  % for internal use
        % account for render times greater than intended delay
        rounddelay = @(x) x*(x >= 0) + 0.01*(x < 0);
        
        % Animation
        for t = 1:length(a_time)
            frame_start= tic;  % dynamic animation time
            obj.set_alpha(alpha_traj(:, :, t));
            obj.plotTF(1);
            frame_time = toc(frame_start);
            pause(rounddelay(pauses(t) - frame_time));
        end
        
    end

    %---------------------------- jacobian ----------------------------
    %
    %   Computes the jacobian between two biped frames.
    %
    %   Can be invoked in the following ways: 
    %
    %   >> Biped.jacobian(a_alpha, a_ref_frame, a_ee_frame)
    %
    %   >> Biped.jacobian(a_alpha, a_ee_frame)
    %
    %   Expected values for a_ref_frame and a_ee_frame:
    %   'TORSO', 'LEFT_FOOT', 'RIGHT_FOOT'
    %
    %   If a_ref_frame is not specified, it defaults to 'TORSO'. Expects
    %   a 3x1 vector of joint positions (a_alpha) and at least one string
    %   specifying a frame. 
    %
    function J = jacobian(obj, a_alpha, a_ref_frame, a_ee_frame)
    %BIPED.JACOBIAN(a_alpha, a_ref_frame, a_ee_frame)
    %
    %   a_alpha:     3x1 vector of joint angles OR 3x2 if FOOT FRAMES
    %   a_ref_frame: 'TORSO', 'LEFT_FOOT', or 'RIGHT_FOOT'
    %                 Defaults to Biped.stance if passed 2 args
    %   a_ee_frame:  'TORSO', 'LEFT_FOOT', 'RIGHT_FOOT'
    
        % Validating user input:
        % check a_alpha size
        if (size(a_alpha, 1) ~= 3 && size(a_alpha, 2) ~= 1)
            error(['a_alpha must be 3x1.\nBiped.jacobian only calculates',
                   ' the jacobian of one foot']);
        end
        if strcmpi(a_ref_frame, a_ee_frame)
            error('Cannot the Jacobian of a frame relative to itself');
        end
        % default a_ref_frame is 'TORSO'
        if nargin == 3
            a_ee_frame = a_ref_frame;
            a_ref_frame = obj.stance;
        end
        % frame input
        fr = obj.stance;              % save stance to avoid overwrite
        obj.set_stance(a_ee_frame);   % verifying end_eff frame
        obj.set_stance(a_ref_frame);  % verifying ref frame
        obj.set_stance(fr);           % return to original stance
        
        % Calculating Jacobian:
        l1 = obj.geom(2);
        l2 = obj.geom(3);
        if strcmpi(a_ref_frame, 'TORSO')  % jacobian of foot relative to torso
            % legs symmetrical, so no need to determine which foot
            a1 = a_alpha(1);
            asum = sum(a_alpha(1:2));
            
            J = [l1*cos(a1)+l2*cos(asum), l2*cos(asum), 0;
                 l1*sin(a1)+l2*sin(asum), l2*sin(asum), 0;
                  1, 1, 1];
        elseif strcmpi(a_ee_frame, 'TORSO')
            error('TORSO jacobian not implemented yet');
        else  % Foot relative to foot
            % Extract angles and positions before the finding the Jacobian
            % for either foot
            error('Foot relative to foot jacobians not implemented yet');
            a1L = a_alpha(1, 1);
            asumL = sum(a_alpha(1:2, 1));
            a1R = a_alpha(1, 2);
            asumR = a_alpha(1:2, 2);
            % Because the position of one foot relative to another is a
            % linear combination of their positions relative to Torso,
            % xRrelL = xLrelTorso - xRrelTorso
            dxL = [l1*cos(a1L)+l2*cos(asumL), l2*cos(asumL), 0];
            dyL = [l1*sin(a1L)+l2*sin(asumL), l2*sin(asumL), 0];
            dxR = [l1*cos(a1R)+l2*cos(asumR), l2*cos(asumR), 0];
            dyR = [l1*sin(a1R)+l2*sin(asumR), l2*sin(asumR), 0];
            
            % Creating Jacobian
            J = zeros(3, 6);  % initialize Jacobian
            J(3, :) = ones(1, 6);  % ankle angle depends on all joints
            
            if strcmpi('RIGHT_FOOT', a_ee_frame)  % RFOOT rel to LFOOT(L-R)
                J(1, 1:3) = dxL;
                J(1, 4:6) = -dxR;
                J(2, 1:3) = dyL;
                J(2, 4:6) = -dyR;
            else  % LFOOT rel to RFOOT
                J(1, 1:3) = -dxL;
                J(1, 4:6) = dxR;
                J(2, 1:3) = -dyL;
                J(2, 4:6) = dyR;
            end
        end
 
    end
    
    
    %---------------------------- ft_inv_kin ----------------------------
    %
    %   Performs geometric inverse kinematics for a foot. Assumes leg
    %   behaves like a human leg where hip angle is on [-pi. pi] and knee
    %   angle is on [-pi, 0].
    %
    %   >> Biped.ft_inv_kin(pos)
    %
    %   pos is a column vector of [x; y; theta] which represents the foot
    %   position and orientation. alpha3 is the angle of the foot relative
    %   to the world frame (so sum(alphas)).
    %
    function alphas = ft_inv_kin(obj, pos)
    %BIPED.FT_INV_KIN Foot inverse kinematics relative to torso frame.
    %   BIPED.FT_INV_KIN(pos) Returns a 3x1 vector of joint angles in
    %   radians which reaches pos where pos = [xdes; ydes; thetades]. The
    %   calculation is biased towards having the knee angle (alpha 2) be
    %   negative. This is to ensure the joint configuration mimics a human
    %   leg.
        
        % extracting spatial position
        de = pos(1:2);
        % converting foot location from torso to hip joint
        de(2) = sign(de(2)) * (abs(de(2)) - obj.geom(1)/2);
        
        % law of cosines
        C  = norm(de);
        A  = obj.geom(2);
        B  = obj.geom(3);
        b  = acos((B^2 - A^2 - C^2) / (-2*A*C));
        c  = acos((C^2 - A^2 - B^2) / (-2*A*B));
        
        % joint angles
        angde = atan2(de(2), de(1));
        th = pi/2 - abs(angde);    % relationship b/t th, b, alpha1
        if (de(1) < 0  && th < 0)
            alpha1 = b - abs(th);  % leg back, foot back
        elseif (de(1) > 0 && th > 0)
            alpha1 = abs(th) - b;  % leg front, foot back
        else
            alpha1 = b + abs(th);  % leg front, foot front
        end
        alpha2 = c - pi;  % assume alpha2 on [-pi, 0]
        
        % organizing/assigning output
        alphas = [alpha1; alpha2; pos(3)-alpha1-alpha2];
        
    end
    
    %---------------------------- rr_inv_kin ----------------------------
    %
    %   Generates a joint trajectory to accomplish a given workspace
    %   trajectory. Assumes leg behaves like a human leg where hip angle 
    %   is on [-pi, pi] and knee angle is on [-pi, 0].
    %
    %   >> Biped.rr_inv_kin(a_traj_ws, a_time, a_ref_frame, a_ee_frame)
    %
    %   a_traj_ws is a 3xT matrix of [x;y;theta] values for a foot at T
    %   time instants. a_time is a 1xT vector of T time instants. 
    %   a_ref_frame is the Biped reference frame. a_ee_frame is the frame
    %   of the end-effector.
    %
    %   Expected values for a_ref_frame and a_ee_frame:
    %   'TORSO', 'LEFT_FOOT', 'RIGHT_FOOT'
    %
    function traj_alpha = rr_inv_kin(obj, a_traj_ws, a_time, a_ref_frame, a_ee_frame)
        
        traj_alpha = zeros(size(a_traj_ws));
        traj_alpha(:, 1) = obj.ft_inv_kin(a_traj_ws(:, 1));
        
        % Finding Body Velocity:
        % don't need to convert a_traj_ws to 
        dtraj = diff(a_traj_ws, 1, 2);
        dt = diff(a_time);
        dtraj_dt = dtraj ./ dt;  % twist velocity
        
        % Integrating Trajectory:
        for ii = 1:length(dt)
            J = obj.jacobian(traj_alpha(:, ii), a_ref_frame, a_ee_frame);
            Jinv = pinv(J);
            dalpha_dt = Jinv * dtraj_dt(:, ii);  % solving for joints' vel
            traj_alpha(:, ii+1) = traj_alpha(:, ii) + dalpha_dt*dt(ii);
        end
        
    end
    
  end     % methods
end

