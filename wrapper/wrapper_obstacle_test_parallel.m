function [cell_coll_result, cell_coll_obs, cell_coll_drones, cell_coll_walls] = ...
    wrapper_obstacle_test_parallel(...
    in_x_centroid, in_y_centroid, in_x_pos, in_y_pos, in_num_drone)
% Function to be used as a wrapper, that encapsulates both the
% obstacle_test and the collision_check functions to produce summary
% metrics regarding simulation collisions.
% This version expects n x m arrays or n x 1 vectors of inputs for n 
% individuals and will run calculations in parallel to try and reduce 
% calculation time.

% Check that inputs match in size
if ~isequal(length(in_x_centroid), length(in_y_centroid), ...
        size(in_x_pos, 1), size(in_y_pos, 1))
    error("Input sizes are not compatible.")
end

num_indiv = length(in_x_centroid);

% Set up output
c = cell(num_indiv, 4);

% Parallel loops
parfor i = 1:num_indiv

    % Run simulation where obstacle placement can be varied
    [out_pos, ~, ~, out_p_swarm, ~, ~] = ...
        obstacle_test(...
        in_x_centroid(i), in_y_centroid(i), in_x_pos(i,:), ...
        in_y_pos(i,:), in_num_drone(i));

    % Run collision check
    [coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
        out_pos, out_p_swarm, out_p_swarm.r_coll);

    c(i,:) = {coll_result, coll_obs, coll_drones, coll_walls};

end

% MATLAB engine should automatically convert these into lists with
% dictionaries
cell_coll_result = c(:,1);
cell_coll_obs = c(:,2);
cell_coll_drones = c(:,3);
cell_coll_walls = c(:,4);

end
