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

function printSpdDistInfo(clientID, vrep, algorithm, vLeft, vRight, dist)
    persistent startTime;
    info_update_tick = 1.0;

    if (isempty(startTime))
        startTime = tic;
    end

    elapsedTime = toc(startTime);            
    if (elapsedTime >= info_update_tick)
        startTime = tic;

        % Fix string to display it
        algstr = pad(algorithm, 17-length(algorithm));
        
        % Prepare the info message
        str = sprintf([
                      '+------------------------------------+ \n'...
                      '|    %s     |  Alg: %s| \n'...        
                      '+-----------------+------------------+ \n'...
                      '| vLeft = %0.4f  |  vRight = %0.4f |  \n'...
                      '+-----------------+------------------+------------------+\n'...
                      '| dist1 = %0.4f  |  dist2 = %0.4f  |  dist3 = %0.4f  |   \n'...
                      '| dist4 = %0.4f  |  dist5 = %0.4f  |  dist6 = %0.4f  |   \n'...
                      '| dist7 = %0.4f  |  dist8 = %0.4f  |  dist9 = %0.4f  |   \n'...
                      '+-----------------+------------------+------------------+\n'], ...
                      datestr(datetime('now'),'HH:MM:SS'), algstr, ...
                      vLeft, vRight, ...
                      dist(1), dist(2), dist(3), ...
                      dist(4), dist(5), dist(6), ...
                      dist(7), dist(8), dist(9));

        % Print info messages on Matlab command window
        fprintf('%s\n', str);

        % Print info messages on V-REP console
        vrep.simxAddStatusbarMessage(clientID, str, vrep.simx_opmode_oneshot);
    end
end

