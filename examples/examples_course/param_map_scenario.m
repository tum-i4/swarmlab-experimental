%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace the original param_map to experiment with cylinder placement for
% swarm simulations.

% Default visualization settings
map.width = 500;
map.max_height = 75;
map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT;
map.bl_corner_north = 0;
map.bl_corner_east = 0;

% Specify arena walls (geofence) here to be read by param_swarm file
map.arena_north = [0 500]; % X wall
map.arena_east = [0 500]; % Y wall
map.arena_down = [-75 -10]; % -Z wall (negative is up)

% Cylinder/building obstacle parameters to be read by param_swarm file
map.building_shape = 'cylinder';
map.buildings_width = [100; 100];
map.buildings_east = [150; 300]; % Corresponds to Y
map.buildings_north = [112.5; 400]; % Corresponds to X
% map.buildings_heights = [100; 100]; % Corresponds to Z

% Specify sphere obstacle parameters here to be read by param_swarm file
map.spheres_north = [250]; % corresponds to X
map.spheres_east = [250]; % corresponds to Y
map.spheres_down = [0]; % corresponds to -Z (negative is up)
map.spheres_r = [100]; % radius [m]
map.n_spheres = length(map.spheres_r);

% Specify blocks here to be read by param_swarm file
map.blocks_width_north = [40 80];
map.blocks_width_east = [60 20];
map.blocks_width_down = [15 20]; 
map.blocks_north = [50 400];
map.blocks_east = [50 400];
map.blocks_down = map.blocks_width_down .* (-1/2); % Put blocks on the floor