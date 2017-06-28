function World = doMapping(World, sen, r_new, scan_data, handles)

    %%%%%%%%%%%%%%%%%%%%%%%%% MAPPING SETTINGS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Amount by which to increment/reduce cell value when obstacle is detected:
    map_augment = 20;
    % Amount by which to reduce cell value when no obstacle is detected:
    map_reduce = 20; 
    map_max = 255;          % Maximum value that a grid cell can take
    map_min = 0;            % Minimum value that a grid cell can take
    map_isOccupied = 150;   % Value at which cell is in occupied state
    map_isEmpty = 50;       % Value at which cell is in empty state
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    scan_polar = getMeasurement(scan_data);
    % Determine the region in the scan that was detected as being emtpy
    empty_region_pol = ones(2, size(scan_polar, 2) * ...
        (round((sen.range - sen.range_min)/World.map_res) + 1)) * -1;
    position = 1;
    for i = 1:size(scan_polar, 2)
        empty_region_r = sen.range_min : World.map_res : scan_polar(1, i) ...
            - World.map_res;
        empty_region_a = repmat(scan_polar(2, i), 1, length(empty_region_r));
        empty_region_pol(:, position : position + length(empty_region_r) - 1) ...
            = [empty_region_r; empty_region_a];
        position = position + length(empty_region_r);
    end
    empty_region_pol = empty_region_pol(:, empty_region_pol(1, :) ~= -1);
    % Map the empty region of the scan onto the global frame
    empty_region = invScanPoint(r_new, empty_region_pol);

    scan_global = transToGlobal(r_new, scan_data);
    gridmap_occupied = zeros(size(World.gridmap));
    gridmap_empty = zeros(size(World.gridmap));
    scan_global = interp1(World.map_vals, World.map_vals, scan_global, 'nearest');
    empty_region = interp1(World.map_vals, World.map_vals, empty_region, 'nearest');

    for i = 1:length(World.map_vals)
        for j = 1:length(World.map_vals)
            if ~isempty(scan_global)
                gridmap_occupied(i, j) = sum(scan_global(1,:) == World.map_vals(j) & ...
                    scan_global(2,:) == World.map_vals(i));
            end
            gridmap_empty(i, j) = sum(empty_region(1,:) == World.map_vals(j) & ...
                empty_region(2,:) == World.map_vals(i));
        end
    end
    
    World.gridmap_counter = World.gridmap_counter ...
                            + map_augment * gridmap_occupied ...
                            - map_reduce  * gridmap_empty;
    
    % Set the gridmap counter such that the max value is 255 and min value is 0:    
    World.gridmap_counter(World.gridmap_counter > map_max) = map_max;
    World.gridmap_counter(World.gridmap_counter < map_min) = map_min;
    
    
    % Let' skip of Image Toolbox dependence
%     World.gridmap_greyscale = mat2gray(World.gridmap_counter, [map_max map_min]);
    mat2grey = @(x) (x - max(x(:))) / (min(x(:)) - max(x(:)));
    World.gridmap_greyscale = mat2grey(World.gridmap_counter);
    
    imagesc(flipud(World.gridmap_greyscale), 'parent', handles.map1, [0 1]);
    set(handles.map1, 'XTick', [], 'YTick', []);
    
end