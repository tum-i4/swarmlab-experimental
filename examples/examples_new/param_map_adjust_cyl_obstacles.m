%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace the original param_map to experiment with cylinder placement for
% swarm simulations.

map.width = 300;
map.max_height = 100;
map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT;
map.bl_corner_north = 0;
map.bl_corner_east = 0;
map.building_shape = 'cylinder';
map.building_width = 37.5;
map.buildings_east = [150; 125; 175; 200]; % Corresponds to Y
map.buildings_north = [112.5; 150; 150; 150]; % Corresponds to X
map.buildings_heights = [100; 100; 100; 150]; % Corresponds to Z
