%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters for the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace the original param_map to experiment with cylinder placement for
% swarm simulations.

% Cylinder obstacle parameters to be read by param_swarm file
map.width = 300;
map.max_height = 100;
map.ACTIVE_ENVIRONMENT = ACTIVE_ENVIRONMENT;
map.bl_corner_north = 0;
map.bl_corner_east = 0;
map.building_width = 37.5;
map.building_shape = 'cylinder';
map.buildings_east = [150; 125; 175; 200]; % Corresponds to Y
map.buildings_north = [112.5; 150; 150; 150]; % Corresponds to X
map.buildings_heights = [100; 100; 100; 150]; % Corresponds to Z

% Specify sphere obstacle parameters here to be read by param_swarm file
map.spheres_north = [200 100]; % corresponds to X
map.spheres_east = [75 200]; % corresponds to Y
map.spheres_down = [-40 -50]; % corresponds to -Z (negative is up)
map.spheres_r = [30 30]; % radius [m]
map.n_spheres = length(map.spheres_r);

% Specify arena walls here to be read by param_swarm file
map.arena_north = [-50 1000]; % X wall
map.arena_east = [0 350]; % Y wall
map.arena_down = [-100 0]; % -Z wall (negative is up)