% Author: Laercio Barbosa 
% e-mail: leoabubauru@hotmail.com
% Github: https://github.com/laercio-barbosa/rmaProject
% Date: 31/05/2017
% 
% SLAM frontend for V-REP .
%
% Based on: mapMakerGUI from Jai Juneja (See Copyright info bellow)
%    - http://www.jaijuneja.com/blog/2013/05/simultaneous-localisation-mapping-matlab/
%    - https://github.com/jaijuneja/ekf-slam-matlab
%
% COPYRIGHT INFO FROM mapMakerGUI
% ===============================
% 
% This software was written and developed by Jai Juneja as part of an 
% undergraduate project in the Department of Engineering Science, University of 
% Oxford. If you have any questions, I can be contacted at: 
% jai.juneja@balliol.ox.ac.uk or jai.juneja@gmail.com
% 
% Please feel free to use, modify and distribute the software for personal or 
% research purposes, along with acknowledgement of the author and inclusion of
% this copyright information.
% 
% Some of the code included has been adapted from other software. They are 
% acknowledged as follows:
% 
% Code for Jacobian transformations was adapted from a SLAM course by Joan Sola 
% (http://www.joansola.eu/JoanSola/eng/course.html)
% ICP algorithm in doICP.m was adapted from code written by Ajmal Saeed Mian 
% (relevant copyright informtion included in the file).
% Any 3rd party code that has been untouched is in the folder 3rd-party

function World = ekfSLAM(handles, AxisDim)
global isSlamRunning DefaultText
    %% %%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    cla; % Clear axes
    
    % Close all figures still openned except the GUI
    figHandles = findobj('type', 'figure', '-not', 'name', 'slamViewerGUI');
    close(figHandles)

    clear performExploration3;
    clear plotExplorationData;
    clear printExplorationInfo2;
    
    disp('Program started');
    
    [clientID, vrep] = initVrep();
    if (clientID > -1)
        disp('Connected to remote API server');
       
        % Advise V-REP we are connected
        vrep.simxAddStatusbarMessage(clientID,'Matlab remote client connected to V-REP!',vrep.simx_opmode_oneshot);
        set(handles.helpBox, 'String', 'Matlab remote client connected to V-REP!')

        % System initialization
        [erro, robotHandle] = initRobot(clientID, vrep, 'Pioneer_p3dx');
        if (vrep.simx_return_ok == erro)
            set(handles.helpBox, 'String', 'Connected to robot!')
        end
            
        [erro, sensorHandle] = initSensor(clientID, vrep);
        if (vrep.simx_return_ok == erro)
            set(handles.helpBox, 'String', 'Connected to all sensors!')
        end
        
        [erro, leftMotorHandle, rightMotorHandle] = initMotors(clientID, vrep, 'Pioneer_p3dx_leftMotor', 'Pioneer_p3dx_rightMotor');
        if (vrep.simx_return_ok == erro)
            set(handles.helpBox, 'String', 'Connected to all motors!')
        end
        
 
        % Laser scanner: LIDAR
        lidar = Sensor;

        % Sensor RF for landmark beacons measurement
        sen_rf = Sensor;
        sen_rf.range       = 5;
        sen_rf.range_min   = 0.01;
        sen_rf.fov         = 2*pi; % It is a RF sensor, so it should be 360 degrees
        sen_rf.noise       = [0.10; 3 * pi/180];
        sen_rf.angular_res = 1 * pi/180;         % 1 degree
        
        % Get robot initial absolut pose to setup the environment
        robotPos = getRobotPos(vrep, clientID, robotHandle);
        [World, rob] = configuration(sen_rf, AxisDim, robotPos);

        Graphics = initGraphics(World, handles);

        %% %%%%%%%%%%%%%%%%%%%%%% TEMPORAL LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Initialise some loop variables
        pose_this_scan = [];
        this_scan_good = 0;
        r = World.r;    % Location of robot pose in mapspace

        % Robot speed and deviation calculation. Runs until simulation has finished!
        t = 1;
        isSlamRunning = true;
        set(handles.helpBox, 'String', 'Executing SLAM...')

        while (vrep.simxGetConnectionId(clientID) ~= -1) && ...
              (t <= World.tend) && ...
              (isSlamRunning == true)
            [vLeft, vRight] = performExploration(clientID, vrep, sensorHandle,robotHandle);

            % Update motors speed [rad/s]
            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle,  vLeft,  vrep.simx_opmode_streaming);
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming);

            World.t = t;

            %% %%%%%%%%%%%%%%%%%%%%%%% SIMULATE WORLD %%%%%%%%%%%%%%%%%%%%%%%%%%
            R_old = rob.R;
            
            % We try to read everithing together in order to avoid the robot
            % moves and we get data from a delayed position
            rob.R = getRobotPos(vrep, clientID, robotHandle);
            
            % Save previous scan and gets new one 
            scan_ref = World.scan_data;            
            [~, World.scan_data] = getLidarData(clientID, vrep);
            
            % Reads the distance of all landmarks
            for lid = 1:size(World.W,2) 
                v = sen_rf.noise .* randn(2,1);
                World.y(:,lid) = scanPoint(rob.R, World.W(:,lid)) + v;
            end
            lmks_all = World.y;
            % Determine landmarks that are within the sensor range
            [World.y, lmks_visible] = sen_rf.constrainMeasurement(World.y);

            %% %%%%%%%%%%%%%%%%%%%%%%%%% START EKF %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                        EKF PREDICTION STEP                           %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% 1. POSE PREDICTION USING SCAN MATCHING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            enough_correlations = 0;

            % Current stored scan is now the last scan
            last_scan_good = this_scan_good;
            pose_last_scan = pose_this_scan;
            this_scan_good = 0; % Initialise this_scan_good for this iteration
            
            % Remove any duplicated measurement
            World.scan_data = unique(World.scan_data','rows')';

            % First add Gaussian white noise to scan
            v = repmat(lidar.noise, 1, size(World.scan_data, 2)) .* randn(2,size(World.scan_data, 2));
            World.scan_data = World.scan_data + v;

            if ~isempty(World.scan_data)
                obstaclesDetected = 1;

                % Convert to Cartesian co-ordinates for scan matching
                World.scan_data = getInvMeasurement(World.scan_data);

                % If the number of point scanned exceeds the specified
                % tolerance, it is suitable for scan matching
                if size(World.scan_data, 2) > World.scan_corr_tolerance
                    this_scan_good = 1;
                end
            else
                obstaclesDetected = 0;
                World.scan_global = [];
            end
            
            % If both last and current scans are good for scan matching,
            % proceed to match scans:
            if last_scan_good && this_scan_good
                n = rob.q .* randn(2,1); % Noise in odometry measurement

                % Do ICP scan matching using odometry input as initial guess
                [R, T, correl, icp_var] = doICP(scan_ref, World.scan_data, 1, rob.u + n);
                
                % If there are enough points correlated between the two
                % scans:
                if length(correl) > World.scan_corr_tolerance
                    enough_correlations = 1;
                    
                    % Determine estimated pose from scan match
                    r_scan = zeros(3, 1);
                    da = getPiToPi(asin(R(2,1)));
                    r_scan(1:2) = transToGlobal(pose_last_scan, T);
                    r_scan(3) = pose_last_scan(3) + da;

                    % Transform scan data to global co-ordinates
                    scan_data_corr = World.scan_data;
                    scan_data_corr = transToGlobal(r_scan, scan_data_corr);
                    
                    % Grab data points that were successfully corellated
                    scan_data_corr = scan_data_corr(:, correl(:, 2)');

                    [r_odo, R_r, R_n] = rob.move(n);

                    % Compute covariance matrix of scan match
                    C = getICPCovariance(icp_var, scan_data_corr);
                    J_u = [R zeros(2, 1); 0 0 1];
                    cov_scan = J_u * C * J_u';
                    cov_scan_norm = trace(cov_scan);
                end
            end

            %% 2. POSE PREDICTION USING ODOMETRY MEASUREMENTS %%%%%%%%%%%%%%%%%%%%%%

            % If there isn't a useful scan, just use odometry data
            if ~enough_correlations
                n = rob.q .* randn(2,1);
                [r_odo, R_r, R_n] = rob.move(n);
                cov_scan = zeros(3, 3);
                dr_scan = zeros(3, 1);
            else
                dr_scan = r_scan - rob.r;
                dr_scan(3) = getPiToPi(dr_scan(3));
            end            

            %% 3. WEIGHTAGE OF ODOMETRY AND SCAN MATCH PREDICTIONS %%%%%%%%%%%%%%%%%

            cov_odo = R_n * World.Q * R_n';
            cov_odo_norm = trace(cov_odo);

            dr_odo = r_odo - rob.r;
            dr_odo(3) = getPiToPi(dr_odo(3));

            dr_true = rob.R - R_old;

            % Check if the robot is making a large turn
            if abs(dr_scan(2)) > pi/6 || abs(dr_odo(2)) > pi/6
                big_turn = 1;
            elseif abs(dr_odo(2)) < pi/18
                big_turn = 0;
            end

            % Determine weights:
            % If not enough correlations, or turning is large, use odometry
            if ~enough_correlations % || big_turn
                weight_odo = 1;
                weight_scan = 0;
            else
                weight_odo = cov_scan_norm / (cov_scan_norm + cov_odo_norm);
                weight_scan = 1 - weight_odo;
                weight_scan = 1;
                weight_odo = 0;
            end

            % Determine robot pose using weights:
            rob.r = weight_odo * dr_odo + weight_scan * dr_scan + rob.r;
            World.x(r) = rob.r;

            % Covariance update
            % Caution: this method of update is suboptimal for CPU processing.
            % Alternative is to have a single equation that updates both
            % the pose and landmark covariances in a single step, but this
            % equation is rather complicated.
            P_rr = World.P(r,r);
            World.P(r,:) = R_r * World.P(r,:);
            World.P(:,r) = World.P(r,:)';
            World.P(r,r) = R_r * P_rr * R_r' + ...
                weight_scan * cov_scan + ...
                weight_odo * cov_odo;

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %                         EKF UPDATE STEP                              %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %% 1. CORRECT KNOWN VISIBLE LANDMARKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Find the visible landmarks. The below code circumvents the data
            % association problem:
            r_old = rob.r;

            lmks_visible_known = find(World.l(1,lmks_visible));
            lids = lmks_visible(lmks_visible_known);

            for lid = lids    
                % Measurement prediction
                [e, E_r, E_l] = scanPoint(rob.r, World.x(World.l(:,lid)));

                E_rl = [E_r E_l];
                rl   = [r World.l(:,lid)'];
                E    = E_rl * World.P(rl,rl) * E_rl';

                % Actual Measurement
                yi = World.y(:,lid);

                % Innovation
                z = yi - e;
                z(2) = getPiToPi(z(2));
                Z = World.M + E;

                % Kalman gain
                K = World.P(:, rl) * E_rl' * Z^-1;

                % Update state and covariance
                World.x = World.x + K * z;
                World.P = World.P - K * Z * K'; % The complexity of this line is 
                                                % very high (subtraction of two
                                                % large matrices)
                rob.r = World.x(r);
            end

            %% 2. INITIALISE NEW LANDMARKS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Check landmark availabiliy.
            % Again data association is avoided. A new landmark is initialised
            % in the storage space reserved specifically for that landmark:
            lmks_visible_unknown = find(World.l(1,lmks_visible)==0);
            lids = lmks_visible(lmks_visible_unknown);

            if ~isempty(lids)
                for lid = lids
                    s = find(World.mapspace, 2);
                    if ~isempty(s)
                        World.mapspace(s) = 0;
                        World.l(:,lid) = s';
                        % Measurement
                        yi = World.y(:,lid);

                        [World.x(World.l(:,lid)), L_r, L_y] = invScanPoint(rob.r, yi);
                        World.P(s,:) = L_r * World.P(r,:);
                        World.P(:,s) = World.P(s,:)'; % Cross variance of lmk with all other lmks
                        World.P(s,s) = L_r * World.P(r,r) * L_r' + L_y * World.M * L_y';
                    end
                end
            end
            pose_this_scan = rob.r; % The corrected pose at which the scan was taken

            %% %%%%%%%%%%%%%%%%%%%%%%%% MAPPING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Determine pose correction during update step
            update = rob.r - r_old;

            if enough_correlations && ~isequal(rob.r, r_old)
                % Adjust latest correlated scan points to account for pose correction
                World.scan_data = transformPoints(World.scan_data, update);

                World = doMapping(World, lidar, rob.r, World.scan_data, handles);
            elseif ~obstaclesDetected && isequal(mod(World.t, 5), 0)
                % If no obstacles have been detected, only compute map every 5
                % iterations
                World = doMapping(World, lidar, rob.r, World.scan_data, handles);
            end

            if ~isempty(World.scan_data)
                World.scan_global = transToGlobal(rob.r, World.scan_data);
            end

            %% %%%%%%%%%%%%%%%%%%%%% HISTORICAL DATA COLECTION %%%%%%%%%%%%%%%%%%%%%
            World.R_hist(:, t)  = rob.R(1:2);   % True position history
            World.r_hist(:, t)  = rob.r(1:2);   % Estimated position history
            pose_error2         = (rob.R(1:2) - rob.r(1:2)).^2;
            World.error_hist(t) = sqrt(pose_error2(1) + pose_error2(2));
            World.Pr_hist(:, t) = [World.P(r(1),r(1)); World.P(r(2),r(2))];

            if enough_correlations
                scan_error    = abs(dr_scan-dr_true); 
                scan_error(3) = abs(getPiToPi(scan_error(3)));
                scan_error(1) = pdist([0 0; scan_error(1) scan_error(2)]); 
                scan_error(2) = [];
            else
                scan_error = [0; 0];
            end

            odo_error    = abs(dr_odo-dr_true);
            odo_error(3) = abs(getPiToPi(odo_error(3)));
            odo_error(1) = pdist([0 0; odo_error(1) odo_error(2)]); 
            odo_error(2) = [];

            World.scan_error_hist(:, t) = scan_error;
            World.odo_error_hist(:, t)  = odo_error;
            World.turning_hist(t)       = rob.u(2);
            World.weight_scan_hist(t)   = weight_scan;
            World.weight_odo_hist(t)    = weight_odo;

            %% %%%%%%%%%%%%%%%%%%%%%%%%%% GRAPHICS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Graphics.lmks_all     = lmks_all; 
            Graphics.lmks_visible = lmks_visible;
            doGraphics(rob, World, Graphics, AxisDim);

            t = t + 1;
        end

        
        % Stop motors if SLAM was stopped by user
        if (vrep.simxGetConnectionId(clientID) ~= -1 || isSlamRunning == false)
            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle,  0, vrep.simx_opmode_streaming);
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, vrep.simx_opmode_streaming);
            set(handles.helpBox, 'String', ['SLAM stopped by user! ' DefaultText])
        else
            set(handles.helpBox, 'String', ['SLAM complete! ' DefaultText]);
        end

        pause(1.0); % Only to show the above messages on GUI! ;)
        isSlamRunning = false;

        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
        
        plotHistoricalData(Graphics, World);
    else
        set(handles.helpBox, 'String', ['Failed connecting to V-REP. Check if the simulation is running. ' DefaultText])
        disp('Failed connecting to V-REP. Check if the simulation is running.');
        World = [];
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end


function plotHistoricalData(Graphics, World)
    %% %%%%%%%%%%%%%%%%%%%%%% PLOT HISTORICAL DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot trajectories;
    set(Graphics.R_hist, 'xdata', World.R_hist(1,:), 'ydata', World.R_hist(2, :))
    set(Graphics.r_hist, 'xdata', World.r_hist(1,:), 'ydata', World.r_hist(2, :))
    
    % Plot position uncertainties
    figure('color', 'white')
    subplot(2, 1, 1)
    plot(1:World.tend, World.error_hist)
    title('EKF SLAM: position error over time')
    xlabel('Time (s)'), ylabel('Position Error (m)')
    subplot(2, 1, 2)
    plot(1:World.tend, sqrt(World.Pr_hist(1, :)), 'r', ...
        1:World.tend, sqrt(World.Pr_hist(2, :)), 'b')
    title('EKF SLAM: position uncertainty over time')
    xlabel('Time (s)'), ylabel('Standard Deviation (m)')
    legend('St. Dev. in x', 'St. Dev. in y')
    
    figure('color', 'white')
    subplot(4, 1, 1)
    AX = plotyy(1:World.tend, World.scan_error_hist(1,:), ...
        1:World.tend, World.scan_error_hist(2,:), 'plot');
    set(get(AX(1),'Ylabel'),'String',['Position' char(10) 'Error (m)'])
    set(get(AX(2),'Ylabel'),'String',['Angular' char(10) 'Error (rad)'])
    title('Scan errors over time')
    xlabel('Time (s)')
    
    subplot(4, 1, 2)
    AX2 = plotyy(1:World.tend, World.odo_error_hist(1,:), ...
        1:World.tend, World.odo_error_hist(2,:), 'plot');
    set(get(AX2(1),'Ylabel'),'String',['Position' char(10) 'Error (m)']) 
    set(get(AX2(2),'Ylabel'),'String',['Angular' char(10) 'Error (rad)'])
    title('Odometry error over time')
    
    subplot(4, 1, 3)
    plot(1:World.tend, World.weight_scan_hist, 'r', ...
        1:World.tend, World.weight_odo_hist, 'b')
    title('Scan and odometry weights over time')
    xlabel('Time (s)'), ylabel('Weight')
    legend('Scan', 'Odometry')
    axis tight
    
    subplot(4, 1, 4)
    plot(1:World.tend, World.turning_hist)
    title('Turning control over time')
    xlabel('Time (s)'), ylabel('Angle (rad)')
    axis tight
end