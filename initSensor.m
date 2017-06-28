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

function [erro, sensorHandle] = initSensor(clientID, vrep)
%INITSENSOR Sensor initialization
%   Connects to V-REP remote server and gets sonar and lidar handles
    sensorHandle = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    erroSensor = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    for i = 1:16
        [erroSensor(i), sensorHandle(i)] = vrep.simxGetObjectHandle(clientID,['Pioneer_p3dx_ultrasonicSensor' num2str(i)],vrep.simx_opmode_oneshot_wait);
        if (vrep.simx_return_ok == erroSensor(i))
            % We start streaming data, but ignore function outputs as
            % is the first call - see function help
            [~, ~, ~, ~] = vrep.simxReadProximitySensor(clientID, sensorHandle(i),vrep.simx_opmode_streaming);
        end
    end
    
    if (~any(erroSensor))  % If there was no errors
        disp('Connected to all ultrasonic sensors!');
        
        % Let's connect to LIDAR. For some reason this frunction returns error
        % if only called with simx_opmode_streaming. Then we make sure it is
        % working with simx_opmode_oneshot_wait and call it again with
        % simx_opmode_streaming to start sending data
        [erro, ~] = vrep.simxGetStringSignal(clientID,'lidarData',vrep.simx_opmode_oneshot_wait);
        if (vrep.simx_return_ok == erro)
            disp('Connected to LIDAR sensors!');
           [~, ~] = vrep.simxGetStringSignal(clientID,'lidarData',vrep.simx_opmode_streaming);
        else
            disp('Handle for LIDAR sensor not found!');
        end
    else
        disp(['Handle for sensor Pioneer_p3dx_ultrasonicSensor' num2str(i) ' not found!']);
        erro = vrep.simx_return_novalue_flag;
    end
end

