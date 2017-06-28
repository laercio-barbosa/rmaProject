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

function [erro, data] = getLidarData(clientID, vrep)
%GETLIDARDATA Get data from LIDAR sensor
%   Gets a vector with the LIDAR data in format [x1 y1 z1 x2 y2 z2 ... xn yn zn] 
%   and convert it to polar form.
    [erro,signal] = vrep.simxGetStringSignal(clientID,'lidarData',vrep.simx_opmode_buffer);
    if(vrep.simx_return_ok == erro)
        dataVec = double(vrep.simxUnpackFloats(signal));
        dataMatrix = reshape(dataVec, 3, []);
        [theta, dist] = cart2pol(dataMatrix(1,:,:), dataMatrix(2,:,:));
        data = [dist; theta];
    else
        data = [];
    end
end

