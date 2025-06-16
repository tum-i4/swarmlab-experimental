function [coll_result, coll_obs, coll_drones, coll_walls] = ...
    wrapper_obstacle_test(...
    in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone)
% Function to be used as a wrapper, that encapsulates both the
% obstacle_test and the collision_check functions to produce summary
% metrics regarding simulation collisions.

% Run simulation
[out_pos, ~, ~, out_p_swarm, ~, ~] = ...
    obstacle_test(...
    in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone);

% Calculate collision stats
[coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
    out_pos, out_p_swarm, out_p_swarm.r_coll);

end