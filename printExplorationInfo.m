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

function printExplorationInfo(clientID, vrep, dist, h, k, vLeft, vRight)
%PRINTEXPLORATIONINFO Summary of this function goes here
%   Detailed explanation goes here
    persistent startTime;
    info_update_tick = 1.0;

    if (isempty(startTime))
        startTime = tic;
    end

    % Print some info/results and plot a graph
    elapsedTime = toc(startTime);            
    if (elapsedTime >= info_update_tick)
        startTime = tic;

        % Prepare the info message
        str = sprintf(['%s  -->  vLeft = %0.10f  |  vRight = %0.10f  |  ', ...
                      'dist = %0.10f  |  h = %0.10f  |  (dist - h) = %0.10f  |  k =  %0.10f'], ...
                      datestr(datetime('now')), vLeft, vRight, dist, h, dist - h, k);

        % Print info messages on Matlab console
        fprintf('%s\n', str);

        % Print info messages on V-REP console
        vrep.simxAddStatusbarMessage(clientID, str, vrep.simx_opmode_oneshot);
    end
end

