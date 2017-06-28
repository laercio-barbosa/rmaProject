% Copyright 2017 Laercio Barbosa
% Author: Laercio Barbosa 
% e-mail: leoabubauru@hotmail.com
% Github: https://github.com/laercio-barbosa/rmaProject
% Date: 31/05/2017
% 
% SLAM frontend for V-REP .
%
% COPYRIGHT INFO
% ==============
% 
% This software was written and developed by Laercio Barbosa as part of an 
% master thesis test in the course Autonomous Mobile Robots from the Instituto de 
% Computação e Matemática Computacional, University of São Paulo, USP, Brazil.
% If you have any questions, I can be contacted at: 
%
% leoabubauru@hotmail.com
% 
% Please feel free to use, modify and distribute the software for personal or 
% research purposes, along with acknowledgement of the author and inclusion of
% this copyright information.
% 
% -------------------------------------------------------------------
% THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
% WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
% AUTHOR, COPPELIA ROBOTICS GMBH AND MATHWORKS WILL NOT BE LIABLE 
% FOR DATA LOSS, DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS
% WHILE USING OR MISUSING THIS SOFTWARE.
% -------------------------------------------------------------------

function [vLeft, vRight] = performExploration(clientID, vrep, sensorHandle, robotHandle)
%PERFORMEXPLORATION Wall following algorithm for exploration
    persistent algorithm  
    
    [dist, detect] = getSonarMeasurements(clientID, vrep, sensorHandle); 
    
    if (isempty(algorithm))
        % Check the surroundings (sensors from 1 to 5 only!!!)
        % Nothing surrounding us?
        surrounding = find(dist(1:6)<0.25, 1);

        if (isempty(surrounding) && ...
            dist(8) <= 0.5 && dist(8) >= 0.05 && ...
            dist(9) <= 0.5 && dist(9) >= 0.05) % Nothing surrounding us and close to the wall
            algorithm = 'Zigzag';
        else
            algorithm = 'Braitenberg';
        end
    end
    
    switch (algorithm)
        case 'Braitenberg'
            [vLeft, vRight, algorithm] = performBraitenberg(dist, detect);

        case 'Lyapunov'
            [vLeft, vRight, algorithm] = performLyapunov(clientID, vrep, dist, detect);
            
        case 'Cornering'
            [vLeft, vRight, algorithm] = performCornering(clientID, vrep, dist, detect, robotHandle);

        case 'Zigzag'
            [vLeft, vRight, algorithm] = performZigzag(dist, detect);
            
        otherwise % Find a wall
            [vLeft, vRight, algorithm] = performFindWall(dist, detect);
    end
    
    printSpdDistInfo(clientID, vrep, algorithm, vLeft, vRight, dist),
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vLeft, vRight, algorithm] = performBraitenberg(dist, detect)
persistent braitenbergL braitenbergR startTime
    
    if (isempty(braitenbergL))
        braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
        braitenbergR = [-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    end
    
    v0 = 2.0;
    stuck_detect_tick = 5.0;

    vLeft  = v0;
    vRight = v0;

    for i = 1:16
        vLeft  = vLeft  + braitenbergL(i) * detect(i);
        vRight = vRight + braitenbergR(i) * detect(i);
    end

    % Are we stucked? If so, let's skip this condition...
    if (vLeft <= 0.0020 && vRight <= 0.0020)
        if (isempty(startTime))
            startTime = tic;
        end
        elapsedTime = toc(startTime);            
        if (elapsedTime >= stuck_detect_tick)
            clear startTime;
            braitenbergL = [-0.6,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,-1.8, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
            braitenbergR = [-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,-0.1,-0.1, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
        end
    elseif (vLeft >= v0/4 && vRight >= v0/4)
        braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
        braitenbergR = [-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
    end
    
    % Check the surroundings (sensors from 1 to 5 only!!!)
    % Nothing surrounding us?
    surrounding = find(dist(1:6)<0.25, 1);

    if (isempty(surrounding) && ...
        dist(8) <= 0.3 && dist(8) >= 0.05 && ...
        dist(9) <= 0.3 && dist(9) >= 0.05) % Nothing surrounding us and close to the wall
        algorithm = 'Zigzag';
        clear braitenbergL; % We guarantee next time the parameters will be restored
    else
        algorithm = 'Braitenberg';
    end
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vLeft, vRight, algorithm] = performZigzag(dist, detect)
    v0 = 2.0;

    % Check the surroundings (sensors from 1 to 5 only!!!)
    % Nothing surrounding us?
    surrounding = find(dist(1:6)<0.25, 1);

    if (isempty(surrounding) && dist(8) > 1.0)
        % Were we following a wall and detected a door or a right turn?
        vLeft  = 0;
        vRight = 0;
        algorithm = 'Cornering';
    elseif (~isempty(surrounding))
        % Is there any risc of collision from sensor 1 to 5?
        vLeft  = v0/2;
        vRight = v0/2;
        algorithm = 'Braitenberg';
    elseif (dist(8) < 0.15 && dist(8) >= 0.05)
        % We are following a wall, but too close of it!
        vLeft  = v0/2;
        vRight = v0;
        algorithm = 'Zigzag';
    elseif (dist(8) > 0.2 && dist(8) <= 0.5 && ...
            dist(6) > 0.22 && dist(7) > 0.22)
        % We are following a wall, but too far from it!
        vLeft  = v0;
        vRight = v0/2;
        algorithm = 'Zigzag';
    else
        vLeft  = v0;
        vRight = v0;
        algorithm = 'Zigzag';
    end
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vLeft, vRight, algorithm] = performCornering(clientID, vrep, dist, detect, robotHandle)
persistent robotInitPos
    v0 = 1.0;
    
    if (isempty(robotInitPos))
        robotInitPos = getRobotPos(vrep, clientID, robotHandle);
    end
    
    % Check the surroundings (sensors from 1 to 5 only!!!)
    % Nothing surrounding us?
    surrounding = find(dist(1:6)<0.25, 1);

    robotPos = getRobotPos(vrep, clientID, robotHandle);

    % Check if the robot rotate 90 degrees
    if (abs(robotPos(3) - robotInitPos(3)) >= pi/2)
        vLeft  = 0.0;
        vRight = 0.0;
        algorithm = 'Zigzag';
        clear robotInitPos;
    elseif (~isempty(surrounding))
        % Is there any risc of collision from sensor 1 to 5?
        vLeft  = v0/2;
        vRight = v0/2;
        algorithm = 'Braitenberg';
    else
        vLeft  =  v0;
        vRight = v0/3;
        algorithm = 'Cornering';
    end
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vLeft, vRight, algorithm] = performLyapunov(clientID, vrep, dist, detect)
    gearRatio = 4.918;
    R = 0.1953/2.0;  % Wheel radius [m] --> From manual in the internet
    ww = 0.0474;     % Wheel widht [m]  --> From manual in the internet
    v = 2.0;         % Linear speed [m/s]
    L = ((0.393-(ww*2))/2.0); % Half of robot width minus wheels width [m]  --> From manual in the internet
    h = L + ww + 0.01;        % Center robot distance to desired path [m]
    k = (v/h) * 0.7; % Gain. 0 < k < v/h so we use 80% from v/h
%    k = 0.3; % Gain. From paper
%     if (((dist(6) < 0.35) && (dist(7) < 0.3)) || (braitenbergIsON == true))
%         vLeft  = v0;
%         vRight = v0;
%         braitenbergIsON = true;
%         if (dist(7) >= 0.3)
%             braitenbergIsON = false;
%         end
% 
%         for i = 1:16
%             vLeft  = vLeft  + braitenbergL(i) * detect(i);
%             vRight = vRight + braitenbergR(i) * detect(i);
%         end
%     if (dist(7) < 0.3)
%         v = 1.0;
%         k = (v/h) * 0.7; % Gain. 0 < k < v/h so we use 80% from v/h
% 
%         % Lets keep the denominator free of zero division or negative number
%         sub = k*(dist(7)-h);
%         if (sub >= v)
%             sub = v * 0.99;
%         end
% 
%         w = (v*power(k,2)*(dist(7)-h)) / sqrt(power(v,2)-power(sub,2)); % Equation 18
%         vRight = (v * R - w * R * L) * gearRatio; % Equation 11
%         vLeft  = (v * R + w * R * L) * gearRatio; % Equation 12
%         printExplorationInfo2(clientID, vrep, dist(8), dist(7), dist(6), h, k, vLeft, vRight);
    if (dist(6) < 0.2)
        v = 1.0;
        k = (v/h) * 0.7; % Gain. 0 < k < v/h so we use 80% from v/h

        % Lets keep the denominator free of zero division or negative number
        sub = k*(dist(6)-h);
        if (sub >= v)
            sub = v * 0.99;
        end

        w = (v*power(k,2)*(dist(6)-h)) / sqrt(power(v,2)-power(sub,2)); % Equation 18
        vRight = (v * R - w * R * L) * gearRatio; % Equation 11
        vLeft  = (v * R + w * R * L) * gearRatio; % Equation 12
        printExplorationInfo2(clientID, vrep, dist(9), dist(8), dist(7), dist(6), h, k, vLeft, vRight);
    else
        % detects a corner
        if (dist(8) > 1.0 || dist(8) < 0.05)
            distFromWall = dist(9);
        else
            distFromWall = dist(8);
        end
        
        % Lets keep the denominator free of zero division or negative number
        sub = k*(dist(8)-h);
        if (sub >= v)
            sub = v * 0.99;
        end

        w = (v*power(k,2)*(distFromWall-h)) / sqrt(power(v,2)-power(sub,2)); % Equation 18
        vRight = (v * R - w * R * L) * gearRatio; % Equation 11
        vLeft  = (v * R + w * R * L) * gearRatio; % Equation 12
        printExplorationInfo2(clientID, vrep, dist(9), dist(8), dist(7), dist(6), h, k, vLeft, vRight);
    end
    
    if ((dist(8) > 0.6 && dist(9) > 0.6) || (dist(8) < 0.05 && dist(9) < 0.05))
        algorithm = 'Cornering';
    elseif ((dist(2) < 0.15 && dist(2) > 0.05) || ...
            (dist(3) < 0.15 && dist(3) > 0.05) || ...
            (dist(4) < 0.15 && dist(4) > 0.05) || ...
            (dist(5) < 0.15 && dist(5) > 0.05))
        % Only switch to Braitenberg if there is some risc of collision, but not
        % given by sensors 6 or 7
        algorithm = 'Braitenberg';
    end
end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dist, detect] = getSonarMeasurements(clientID, vrep, sensorHandle)
    % Scene and pioneer motion variables
    noDetectionDist  = 0.5;  % No  detection distance
    maxDetectionDist = 0.05; % Max detection distance
    detect = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    dist   = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    
    for i = 1:16
        [erro, state, coord, ~, ~] = vrep.simxReadProximitySensor(clientID, sensorHandle(i),vrep.simx_opmode_buffer);
        if (erro == vrep.simx_return_ok && state == true)
            dist(i) = sqrt(power(coord(1),2) + power(coord(2),2) + power(coord(3),2));
            if (state == true && dist(i) < noDetectionDist)
                if (dist(i) < maxDetectionDist)
                    dist(i) = maxDetectionDist;
                end
                detect(i) = 1 - ((dist(i) - maxDetectionDist) / (noDetectionDist - maxDetectionDist));
            else
                detect(i) = 0;
            end
        elseif (state == false)
            detect(i) = 0;
            dist(i) = 1.1111;
        else
            detect(i) = 0;
            dist(i) = 5.5555;
        end
    end
    
    % Fix infinite values
    dist(dist==Inf) = 9.9999;
end