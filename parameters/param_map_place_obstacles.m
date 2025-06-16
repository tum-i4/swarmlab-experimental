function [map] = param_map_place_obstacles(in_x_vec, in_y_vec)
% Function to set map parameters, including map size and obstacle
% placement.

% Specify arena walls here to be read by param_swarm file
map.arena_north = [-50 350]; % X wall
map.arena_east = [0 350]; % Y wall
map.arena_down = [-100 0]; % -Z wall (negative is up)

map.width = 300;
map.max_height = 100;
map.ACTIVE_ENVIRONMENT = true;
map.bl_corner_north = 0;
map.bl_corner_east = 0;
%map.building_width = 37.5;
map.building_width = 37.5;
map.building_shape = 'cylinder';
map.buildings_east = in_y_vec'; % Corresponds to Y
map.buildings_north = in_x_vec'; % Corresponds to X
map.buildings_heights = zeros(length(in_x_vec),1) + 100; % Corresponds to Z
