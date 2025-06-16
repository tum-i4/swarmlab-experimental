function [pos_ned_hist, vel_ned_hist, time_vec, p_swarm, map, alg_conn] = ...
    obstacle_test(...
    in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone)
% Function to be used as wrapper, to be called by python script for
% scenario-based testing searches.
% Inputs:
%   in_x_centroid   = [scalar] centroid of swarm, x/north
%   in_y_centroid   = [scalar] centroid of swarm, y/east
%   in_x_vec        = [vector] x-position of cylindrical obstacles
%   in_y_vec        = [vector] y-position of cylindrical obstacles
%   in_num_drone    = [scalar] number of drones
%
% Outputs:
%   pos_ned_hist    = [mat] position history in north/east/down
%   vel_ned_hist    = [mat] velocity history in north/east/down
%   p_swarm         = [struct] contains swarm parameters
%   map             = [struct] contains map parameters
%   alg_conn        = [vector] history of algebraic connectivity


% Ensure vector inputs are rows
if ~isrow(in_x_vec)
    in_x_vec = in_x_vec';
end
if ~isrow(in_y_vec)
    in_y_vec = in_y_vec';
end

% Change to home directory and add all subdirectories to active path
homeDir = fullfile(fileparts(mfilename('fullpath')), '..');
cd(homeDir);
addpath(genpath(pwd));

% swarming mode supports only quadcopter and point_mass
DRONE_TYPE = "point_mass"; 
% DRONE_TYPE = "quadcopter";

% Set number of drones in swarm
p_swarm.nb_agents = in_num_drone; % (will default to 5 in param_swarm if not set here)

% This is used to set up obstacle parameters in param_swarm (ignore the
% warning)
ACTIVE_ENVIRONMENT = true; 

% Set swarm control algorithm
SWARM_ALGORITHM = "vasarhelyi"; % either vasarhelyi or olfati_saber
ACTIVE_ARENA_WALLS = true;

% Set simulation parameters
run('param_sim');
run('param_battery');
run('param_physics');
run('param_drone');

% Sets map parameters, including map size and obstacle placement
map = param_map_place_obstacles(in_x_vec, in_y_vec);

% Most parameters of interest are set here:
%   - Seed
%   - Arena parameters
run('param_swarm'); 

% Check config parameters and set defaults if needed
[p_sim, p_battery, p_physics, p_drone, map, p_swarm] = ...
    check_params(p_sim, p_battery, p_physics, p_drone, map, p_swarm);

% Set swarm starting position
p_swarm.P0 = [in_x_centroid,in_y_centroid,-50]'; % [m] position of a vertex of the cube
p_swarm.Pos0 = p_swarm.P0 + p_swarm.P * rand(3,p_swarm.nb_agents);
p_swarm.is_active_arena = ACTIVE_ARENA_WALLS;
p_swarm.x_arena = [map.arena_north; map.arena_east; map.arena_down];
p_swarm.center_arena = sum(p_swarm.x_arena, 2) / 2;

% Initialize swarm object
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;
for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics, map, p_swarm);
end
swarm.set_pos(p_swarm.Pos0);

% Wind settings: steady wind (1:3), wind gusts (3:6)
wind = zeros(6,1); % Currently set to no wind

% x0 = [p_swarm.Pos0; zeros(3,p_swarm.nb_agents)];
% x_history(1,:) = x0(:);

% Main simulation loop
time_vec = p_sim.start_time:p_sim.dt:p_sim.end_time;
for time = time_vec
    [~,collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);
    swarm.update_state(wind, time);
end
time_vec = time_vec'; % make it a column vector

% Outputs
pos_ned_hist = swarm.get_pos_ned_history();
pos_ned_hist = pos_ned_hist(2:end,:);
vel_ned_hist = swarm.get_vel_xyz_history();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% results_dirname = '';
% accel_history = [zeros(1, p_swarm.nb_agents*3); ...
%     diff(vel_ned_history,1)/p_sim.dt];
% time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
% plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
%     accel_history, [], p_swarm, map, 12, [], ...
%     results_dirname);
[safety, order, union, alg_conn, safety_obs, min_d_obs] = ...
    compute_swarm_performance(pos_ned_hist, vel_ned_hist, ...
    p_swarm, '');
% [perf_handle] = plot_swarm_performance(time_history', safety, order, ...
%        union, alg_conn, safety_obs, min_d_obs, p_swarm, 12, results_dirname);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
