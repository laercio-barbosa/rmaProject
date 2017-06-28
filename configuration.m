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

function [World, rob] = configuration(sen_rf, AxisDim, robotPos)
global RunTime;

    % If you want to make any changes to the default robot/sensor settings
    % then do that here - the changes will be passed back into the main
    % SLAM function.

    % NOTE: LAERCIO - Uses known and given landmarks
    load('lm.mat');
    Lmks=lm;
%     Lmks = robotPos(1:2); TODO: Laercio --> Try to implement real lanemarks
    
    % Robot
    rob        = Robot;         % Create new robot object   
    rob.R(1:2) = robotPos(1:2); % Set robot at first waypoint
    rob.R(3)   = robotPos(3);
    rob.r      = rob.R;         % Initial pose estimation is rthe real position
    
    % These values are set by an engineer to match the system:
    World.Q = diag(rob.q .^ 2);     % System uncertainty
    World.M = diag(sen_rf.noise .^ 2); % Measurement uncertainty
    
    World.tend = RunTime;

    % NOTE: Removed by Laercio
    % Initialise landmarks and waypoints
    World.W = Lmks;
    
    % Initialise measurement vector
    World.y = zeros(2,size(World.W,2));

    % Initialise state and covariance vectors
    World.x = zeros(numel(rob.r)+numel(World.W), 1);    % State estimate
    World.P = zeros(numel(World.x),numel(World.x));     % Covariance matrix
    World.mapspace = 1:numel(World.x);                  % State map availability
    World.l = zeros(2, size(World.W,2));                % Landmark positions

    % Find and reserve mapspace for robot pose
    World.r = find(World.mapspace,numel(rob.r));
    World.mapspace(World.r) = 0;
    World.x(World.r) = rob.R;       % Set initial pose estimate equal to truth
    World.P(World.r, World.r) = 0;  % Set initial pose covariance equal to 0
    
    % Initialise historical vectors
    World.R_hist = zeros(2, World.tend);
    World.r_hist = zeros(2, World.tend);
    World.Pr_hist = zeros(2, World.tend);
    World.error_hist = zeros(1, World.tend);
    World.scan_error_hist = zeros(2, World.tend);
    World.odo_error_hist = zeros(2, World.tend);
    World.turning_hist = zeros(1, World.tend);
    World.weight_scan_hist = zeros(1, World.tend);
    World.weight_odo_hist = zeros(1, World.tend);

    %%%%%%%%%%%%%%%%%%%%%%%%% MAPPING SETTINGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Map resolution (grid size)
    World.map_res = 0.1;
    % Scan correlation tolerance. This defines the minimum number of
    % correlations required for a scan match to be accepted
    World.scan_corr_tolerance = 20;
    % Discretised grid map values
    World.map_vals = -AxisDim+World.map_res:World.map_res:AxisDim-World.map_res;
    World.gridmap = zeros(AxisDim * 2 / World.map_res);
    World.gridmap_counter = ones(size(World.gridmap)) * round(255/2);
    World.scan_data = [];
    World.scan_global = [];
    World.scan_true = [];
end