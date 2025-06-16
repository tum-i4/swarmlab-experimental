% Test wrapper for running simulation in parallel

clear

% Change to home directory and add all subdirectories to active path
homeDir = fullfile(fileparts(mfilename('fullpath')), '..');
cd(homeDir);
addpath(genpath(pwd));

% Set up vectors and matrices of inputs
num_indiv = 20;
in_x_centroid = zeros(num_indiv, 1);
in_y_centroid = 190.309.*ones(num_indiv, 1);
in_x_coords = repmat([211.8, 120.44, 147.496], num_indiv, 1);
in_y_coords = repmat([50.6859, 74.4602, 193.871], num_indiv, 1);
in_num_drone = 5.*ones(num_indiv, 1);

% Set up output
c1 = cell(num_indiv, 4);

% Parallel loops
tic
[cell_coll_result, cell_coll_obs, cell_coll_drones, cell_coll_walls] = ...
    wrapper_obstacle_test_parallel(...
    in_x_centroid, in_y_centroid, in_x_coords, in_y_coords, in_num_drone);
fprintf('Parallel processing of %d individuals.\n', num_indiv)
toc

% Serial for comparison
% Set up output
c2 = cell(num_indiv, 4);

% Serial loops
tic
for i = 1:num_indiv

    % Run simulation where obstacle placement can be varied
    [out_pos, ~, ~, out_p_swarm, ~, ~] = ...
        obstacle_test(...
        in_x_centroid(i), in_y_centroid(i), in_x_coords(i,:), ...
        in_y_coords(i,:), in_num_drone(i));

    % Run collision check
    [coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
        out_pos, out_p_swarm, out_p_swarm.r_coll);

    c2(i,:) = {coll_result, coll_obs, coll_drones, coll_walls};

end
fprintf('Serial processing of %d individuals.\n', num_indiv)
toc
