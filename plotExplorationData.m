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

function [ output_args ] = plotExplorationData(clientID, vrep, dist, h)
%PLOTEXPLORATIONDATA Summary of this function goes here
%   Detailed explanation goes here
    persistent startTime;
    persistent x_axis;
    info_update_tick = 1.0;

    if (isempty(startTime))
        % Setup the graphic only once
        figure(1);
        title('Asymptotic distance stability');
        ylabel('Distance [m]');
        xlabel('Time [s]');
        ylim([-0.2, 0.5]);
        xlim([0, 240]);
        x_axis = 0;
        startTime = tic;
    end

    % Print some info/results and plot a graph
    elapsedTime = toc(startTime);            
    if (elapsedTime >= info_update_tick)
        startTime = tic;

        % Update de graphic
        x_axis = x_axis + info_update_tick;
        hold on;
        plot(x_axis, dist, 'b.', 'DisplayName', 'y = dist');
        plot(x_axis, dist-h, 'r.', 'DisplayName', 'y = dist - h');
        legend('show');
        hold off;
    end
end

