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

function rmaProject()
%RMAPROJECT V-REP remote control for a Pioneer robot
%   Controls Pioneer robot using Braitenberg algorithm in a V-REP simulation

    disp('Program started');
    [clientID, vrep] = initVrep();
    
    if (clientID > -1)
        disp('Connected to remote API server');
        
        % Advise V-REP we are connected
        vrep.simxAddStatusbarMessage(clientID,'Matlab remote client connected to V-REP!',vrep.simx_opmode_oneshot);
        
        % System initialization
        [leftMotorHandle, rightMotorHandle] = initMotors(clientID, vrep, 'Pioneer_p3dx_leftMotor', 'Pioneer_p3dx_rightMotor');
        sensorHandle = initSensor(clientID, vrep);

        % Robot speed and deviation calculation. Runs until simulation has finished!
        while (vrep.simxGetConnectionId(clientID) ~= -1)
            [vLeft, vRight] = performExploration(clientID, vrep, sensorHandle);

            % Update motors speed
            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, vrep.simx_opmode_streaming);
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming);
        end        
        
        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end

