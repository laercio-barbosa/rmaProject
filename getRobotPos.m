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

function [robotPos] = getRobotPos(vrep, clientID, robotHandle)
%GETROBOTPOS Gets the current robot position and orientation
%   Gets the current robot position and orientation x y a [m m rad]
    [~, robotXYZ] = vrep.simxGetObjectPosition(clientID, robotHandle, -1, vrep.simx_opmode_buffer);
    [~, robotAng] = vrep.simxGetObjectOrientation(clientID, robotHandle, -1, vrep.simx_opmode_buffer);
    robotPos = robotXYZ(1:2);
    robotPos = [robotPos robotAng(3)];
    robotPos = robotPos'; % [x y a]' [m m rad]
end

