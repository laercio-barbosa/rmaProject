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
%   Controls Pioneer robot in a V-REP simulation

    close all;
    clear performExploration3;
    clear plotExplorationData;
    clear printExplorationInfo2;
    
    disp('Program started');
    
    [clientID, vrep] = initVrep();
    if (clientID > -1)
        disp('Connected to remote API server');
        
        % Advise V-REP we are connected
        vrep.simxAddStatusbarMessage(clientID,'Matlab remote client connected to V-REP!',vrep.simx_opmode_oneshot);
        
        % System initialization
        [~, leftMotorHandle, rightMotorHandle] = initMotors(clientID, vrep, 'Pioneer_p3dx_leftMotor', 'Pioneer_p3dx_rightMotor');
        [~, sensorHandle] = initSensor(clientID, vrep);
        [~, robotHandle] = initRobot(clientID, vrep, 'Pioneer_p3dx');
        
        % Robot speed and deviation calculation. Runs until simulation has finished!
        while (vrep.simxGetConnectionId(clientID) ~= -1)
            [vLeft, vRight] = performExploration3(clientID, vrep, sensorHandle, robotHandle);

            % Saturate angular velocity if necessary
            if (vLeft > 2.0)
                vLeft = 2.0;
            end
            if (vRight > 2.0)
                vRight = 2.0;
            end
            
            % Update motors speed [rad/s]
            vrep.simxSetJointTargetVelocity(clientID, leftMotorHandle,  vLeft,  vrep.simx_opmode_streaming);
            vrep.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, vrep.simx_opmode_streaming);

            pause(0.01);
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

