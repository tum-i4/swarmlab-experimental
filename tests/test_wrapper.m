% Test wrapper for running simulation and creating a figure

clear

% Change to home directory and add all subdirectories to active path
homeDir = fullfile(fileparts(mfilename('fullpath')), '..');
cd(homeDir);
addpath(genpath(pwd));

% Set up scenario where a collision should occur
% (check param_sim; collision should occur if dt is 0.01 or 0.1)
in_x_centroid = 0; in_y_centroid = 190.309;
in_x_vec = [211.8; 120.44; 147.496]; 
in_y_vec = [50.6859; 74.4602; 193.871];
in_num_drone = 5;

% Run simulation where obstacle placement can be varied
[out_pos, out_vel, out_time, out_p_swarm, out_map, out_alg_conn] = ...
    obstacle_test(...
    in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone);

% Plot result
plot_case(out_pos, out_p_swarm, out_map)

% Run collision check again to show its outputs (this is also run within 
% plot_case)
[coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
    out_pos, out_p_swarm, out_p_swarm.r_coll);

% Also try a run without a collision
in_x_centroid = 0; in_y_centroid = 180;
in_x_vec = [200; 120; 140]; 
in_y_vec = [50; 70; 190];
in_num_drone = 5;

[out_pos, out_vel, out_time, out_p_swarm, out_map, out_alg_conn] = ...
    obstacle_test(...
    in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone);

plot_case(out_pos, out_p_swarm, out_map)
