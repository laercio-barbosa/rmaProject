% Copyright 2017 Laercio Barbosa 
% leoabubauru@hotmail.com
% 
% -------------------------------------------------------------------
% THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
% WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
% AUTHOR AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
% DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
% MISUSING THIS SOFTWARE.
% 
% You are free to use/modify/distribute this file for whatever purpose!
% -------------------------------------------------------------------
%

% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simExtRemoteApiStart(19999)
%
% then start simulation, and run this matlab program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function teste()
%RMAPROJECT V-REP remote control for a Pioneer robot
%   Controls Pioneer robot using Braitenberg algorithm in a V-REP simulation
persistent hasStarted;

    if (isempty(hasStarted) == true)
        hasStarted = false;
    end

    if (hasStarted == false)
        hasStarted = true;
        
        % Scene and pioneer motion variables
        sensorHandle = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
        noDetectionDist  = 0.5; % No  detection distance
        maxDetectionDist = 0.2; % Max detection distance
        detect = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
        braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
        braitenbergR = [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
        v0 = 2;

        disp('Program started');
        % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
        vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
        vrep.simxFinish(-1); % just in case, close all opened connections
        clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

        if (clientID > -1)
            disp('Connected to remote API server');
            % Now send some data to V-REP in a non-blocking fashion:
            vrep.simxAddStatusbarMessage(clientID,'Matlab remote client connected to V-REP!',vrep.simx_opmode_oneshot);

            % Motors initialization
            [erro, leftMotorHandle] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait);
            if (erro ~= 0)
                disp('Handle for left motor not found!');
            else
                disp('Connected to left motor!');
            end

            [erro, rightMotorHandle] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait);
            if (erro ~= vrep.simx_return_ok)
                disp('Handle for right motor not found!');
            else
                disp('Connected to right motor!');
            end

            % Sensors initialization (remoteApi)
            for i = 1:16
                [erro, sensorHandle(i)] = vrep.simxGetObjectHandle(clientID,['Pioneer_p3dx_ultrasonicSensor' num2str(i)],vrep.simx_opmode_oneshot_wait);
                if (erro ~= vrep.simx_return_ok)
                    disp(['Handle for sensor Pioneer_p3dx_ultrasonicSensor' num2str(i) ' not found!']);
                else
                    disp(['Connected to sensor Pioneer_p3dx_ultrasonicSensor' num2str(i) '!']);
                    % We start streaming data, but ignore function outputs as
                    % is the first call - see function help
                    [~, ~, ~, ~] = vrep.simxReadProximitySensor(clientID, sensorHandle(i),vrep.simx_opmode_streaming);
                end
            end
        else
            disp('Failed connecting to remote API server');
        end
    end        

    % Robot speed and deviation calculation. Runs until simulation has finished!
    if (vrep.simxGetConnectionId(clientID) ~= -1)
        for i = 1:16
            [erro, state, coord, ~, ~] = vrep.simxReadProximitySensor(clientID, sensorHandle(i),vrep.simx_opmode_buffer);
            if (erro == vrep.simx_return_ok)
                dist = sqrt(power(coord(1),2) + power(coord(2),2) + power(coord(3),2));
                if (state == true && dist < noDetectionDist)
                    if (dist < maxDetectionDist)
                        dist = maxDetectionDist;
                    end
                    detect(i) = 1 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist));
                else
                    detect(i) = 0;
                end
            else
                detect(i) = 0;
            end
        end

        vLeft  = v0;
        vRight = v0;

        for i = 1:16
            vLeft  = vLeft  + braitenbergL(i) * detect(i);
            vRight = vRight + braitenbergR(i) * detect(i);
        end

        % Update motors speed
        vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming);
    else

        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);

        disp('Program ended');
    end
end
