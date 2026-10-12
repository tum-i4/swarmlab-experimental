%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace the original param_map to experiment with cylinder placement for
% swarm simulations.
% This version shrinks the map size by an order of magnitude compared to
% in param_map_adjust_cyl_obstacles.m

map.width = 30.0;
map.max_height = 10.0;
map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT;
map.bl_corner_north = 0;
map.bl_corner_east = 0;
map.building_width = 3.75;
map.building_shape = 'cylinder';
map.buildings_east = [15.0; 12.5; 17.5; 20.0]; % Corresponds to Y
map.buildings_north = [11.25; 15.0; 15.0; 15.0]; % Corresponds to X
map.buildings_heights = [10.0; 10.0; 10.0; 15.0]; % Corresponds to Z
