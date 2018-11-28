% Author: Laercio Barbosa 
% e-mail: leoabubauru@hotmail.com
% Github: https://github.com/laercio-barbosa/rmaProject
% Date: 31/05/2017
% 
% SLAM frontend for V-REP .
%
% Based on: mapMakerGUI from Jai Juneja (See Copyright info bellow)
%    - http://www.jaijuneja.com/blog/2013/05/simultaneous-localisation-mapping-matlab/
%    - https://github.com/jaijuneja/ekf-slam-matlab
%
% COPYRIGHT INFO FROM mapMakerGUI
% ===============================
% 
% This software was written and developed by Jai Juneja as part of an 
% undergraduate project in the Department of Engineering Science, University of 
% Oxford. If you have any questions, I can be contacted at: 
% jai.juneja@balliol.ox.ac.uk or jai.juneja@gmail.com
% 
% Please feel free to use, modify and distribute the software for personal or 
% research purposes, along with acknowledgement of the author and inclusion of
% this copyright information.
% 
% Some of the code included has been adapted from other software. They are 
% acknowledged as follows:
% 
% Code for Jacobian transformations was adapted from a SLAM course by Joan Sola 
% (http://www.joansola.eu/JoanSola/eng/course.html)
% ICP algorithm in doICP.m was adapted from code written by Ajmal Saeed Mian 
% (relevant copyright informtion included in the file).
% Any 3rd party code that has been untouched is in the folder 3rd-party
%
% Sensor class contains properties and functions relating to the sensors.
classdef Sensor < handle
    properties
        % Sensor properties
        range           % Range of laser sensor (metres)
        range_min       % Minimum distance of sensor reading
        fov             % Angle over which sensor operates (field of view)
        noise           % Measurement noise (range and bearing)
        angular_res     % Angular resolution (separation between readings)
    end
    
    methods
        
        function sen = Sensor                       % Constructor method
            % Default values based on Hokuyo URG-04LX-UG01 datasheet 
            % (website info is different and probably wrong!)
            % The number 684 is the number of points from V-REP sensor.
            sen.range       =   4.095;
            sen.range_min   =   0.06;
            sen.fov         =   240 * pi/180;
            sen.noise       =   [0.01; 0.01];   % 1%, 1%
            sen.angular_res =   (240.0/684.0) * pi/180.0;         % 0.351 degree
        end
        
        % Find all measurements that are out of the sensor's range and set
        % them to an impossible value so that they can be removed
        function [y, idx] = constrainMeasurement(sen, y)
            off_range = y(1, :) > sen.range | y(1, :) < sen.range_min;
            off_ang = y(2, :) > sen.fov/2 | ...
                y(2, :) < -sen.fov/2;
            y(:, off_range) = inf;
            y(:, off_ang) = inf;
            idx = find(y(1, :) < inf | y(2, :) < inf);
        end
        
        % Generate a scan where the full sweep gives the max range value
        function y = generateEmptyScan(sen)
            readings_a = -sen.fov/2 : sen.angular_res : sen.fov/2;
            numLasers = length(readings_a);
            y = [repmat(sen.range, 1, numLasers); readings_a];
        end

    end
end