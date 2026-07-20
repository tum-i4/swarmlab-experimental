function plot_case_with_collisions(pos_ned_history, time_history, p_swarm, map, fail_config, collision)
% Creates figure of simulation environment and drone trajectories. Checks
% for collisions (with obstacle, b/t agents, with wall) and shows where the
% first one occurs, if one exists.
% (note that after figure has been created, it can be manually rotated to
% see 3D perspective)

% results_dirname = strcat('../../../../safetytesting_collabuavs/', in_dir_name);
% if ~exist(results_dirname, 'dir')
%      mkdir(results_dirname);
% end

tol_time = 0.001;

% Create figure - plot trajectories
agents_color = []; % use default
fontsize = 12; % use default
traj_handle = plot_trajectories_offline(pos_ned_history, ...
    p_swarm.nb_agents, agents_color, fontsize, map, p_swarm);

% Iterate through each agent and add markings if...
%   - Collision occurs
%   - Failure occurs
for i = 1:p_swarm.nb_agents

    % Index for drone positions in pos_ned_history matrix
    temp_ind = (3*(i-1))+1;
    
    % Find time when first collision occurs and add marker to figure
    temp_coll_ind = find(collision(:,i), 1);
    if ~isempty(temp_coll_ind)
        figure(traj_handle)
        plot3(pos_ned_history(temp_coll_ind, temp_ind+1), ...
            pos_ned_history(temp_coll_ind, temp_ind), ...
            -pos_ned_history(temp_coll_ind, temp_ind+2), ...
            'o', 'Color', 'b', 'MarkerSize', 10);
    end

    % Find time when failure occurs and add marker to figure
    if i == fail_config.fail_agent_num
        temp_fail_ind = find(abs(time_history-fail_config.fail_time) < tol_time, 1);
        if ~isempty(temp_fail_ind)
            figure(traj_handle)
            plot3(pos_ned_history(temp_fail_ind, temp_ind+1), ...
                pos_ned_history(temp_fail_ind, temp_ind), ...
                -pos_ned_history(temp_fail_ind, temp_ind+2), ...
                'x', 'Color', 'r', 'MarkerSize', 10);
        end
    end

end

% % Check for collisions
% coll_dist = p_swarm.r_coll; % Using controller collision distance as threshold
% [coll_result, ~, ~, ~] = collision_check(...
%     pos_ned_history, p_swarm, coll_dist);
% 
% % If collision occurred, mark drone locations when collision happened
% if coll_result.occur
%     for agent = 1:p_swarm.nb_agents
% 
%         % Index for drone position
%         temp_ind = (3*(agent-1))+1;
% 
%         % Make figure active and plot to it (note axes flipped)
%         figure(traj_handle)
%         plot3(pos_ned_history(coll_result.index, temp_ind+1), ...
%             pos_ned_history(coll_result.index, temp_ind), ...
%             -pos_ned_history(coll_result.index, temp_ind+2), ...
%             'o', 'Color', 'b', 'MarkerSize', 10);
% 
%     end
% end

end
