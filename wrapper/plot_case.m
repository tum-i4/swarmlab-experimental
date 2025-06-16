function plot_case(pos_ned_history, p_swarm, map)
% Creates figure of simulation environment and drone trajectories. Checks
% for collisions (with obstacle, b/t agents, with wall) and shows where the
% first one occurs, if one exists.
% (note that after figure has been created, it can be manually rotated to
% see 3D perspective)

% Create figure - plot trajectories
agents_color = []; % use default
fontsize = 12; % use default
traj_handle = plot_trajectories_offline(pos_ned_history, ...
    p_swarm.nb_agents, agents_color, fontsize, map, p_swarm);

% Check for collisions
coll_dist = p_swarm.r_coll; % Using controller collision distance as threshold
[coll_result, ~, ~, ~] = collision_check(...
    pos_ned_history, p_swarm, coll_dist);

% If collision occurred, mark drone locations when collision happened
if coll_result.occur
    for agent = 1:p_swarm.nb_agents

        % Index for drone position
        temp_ind = (3*(agent-1))+1;

        % Make figure active and plot to it (note axes flipped)
        figure(traj_handle)
        plot3(pos_ned_history(coll_result.index, temp_ind+1), ...
            pos_ned_history(coll_result.index, temp_ind), ...
            -pos_ned_history(coll_result.index, temp_ind+2), ...
            'o', 'Color', 'b', 'MarkerSize', 10);

    end
end
