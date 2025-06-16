%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace the original param_map to experiment with cylinder placement for
% swarm simulations.

% Default visualization settings
map.width = 500;
map.max_height = 150;
map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT;
map.bl_corner_north = 0;
map.bl_corner_east = 0;

% Specify arena walls (geofence) here to be read by param_swarm file
map.arena_north = [0 500]; % X wall
map.arena_east = [0 500]; % Y wall
map.arena_down = [-150 0]; % -Z wall (negative is up)

% Cylinder/building obstacle parameters to be read by param_swarm file
map.building_shape = 'cylinder';
map.buildings_width = [100; 50; 25];
map.buildings_east = [220; 125; 200]; % Corresponds to Y
map.buildings_north = [300; 325; 200]; % Corresponds to X
% map.buildings_heights = [100; 100]; % Corresponds to Z

% Specify sphere obstacle parameters here to be read by param_swarm file
map.spheres_north = [700]; % corresponds to X
map.spheres_east = [700]; % corresponds to Y
map.spheres_down = [0]; % corresponds to -Z (negative is up)
map.spheres_r = [550]; % radius [m]
map.n_spheres = length(map.spheres_r);

% Specify blocks here to be read by param_swarm file
map.blocks_width_north = [40 60 40 100 68 80 60 30];
map.blocks_width_east = [40 40 40 40 68 40 220 60];
map.blocks_width_down = [20 50 30 75 165 60 30 30]; 
map.blocks_north = [40 110 180 70 230 340 130 20];
map.blocks_east = [300 40 40 100 100 100 250 200];
map.blocks_down = map.blocks_width_down .* (-1/2); % Put blocks on the floor