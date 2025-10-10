function [agent_agent, agent_wall, agent_cyl, agent_sph] = ...
    calculate_all_distances(pos_ned, map, r_coll)
% CALCULATE_ALL_DISTANCES Calculate distances between agents, walls, and 
% obstacles for all timesteps, plus collision detection.
%
% Inputs:
%   pos_ned - Position history [time x (3*num_agents)] where each row is 
%             [x1 y1 z1 x2 y2 z2 ... xN yN zN] for N agents
%   map - Environment structure with walls and obstacles
%   r_coll - Agent collision radius: scalar (same for all) or vector [num_agents x 1]
%
% Outputs (structured with .dist, .coll, and .min fields):
%   agent_agent - Agent-agent interactions
%     .dist - [time x num_agents x num_agents] distances between agents
%             agent_agent.dist(t,j,i) = distance from agent i to agent j at time t  
%     .coll - [time x num_agents] logical, true if agent has agent-agent collision
%     .min - [num_agents x num_agents] minimum distance each agent achieved to each other agent
%   agent_wall - Agent-wall interactions
%     .dist - [time x 6 x num_agents] distances to 6 arena walls
%             Walls: 1=North_min, 2=North_max, 3=East_min, 4=East_max, 5=Down_min, 6=Down_max
%     .coll - [time x num_agents] logical, true if agent has wall collision
%     .min - [num_agents x 6] minimum distance each agent achieved to each wall
%   agent_cyl - Agent-cylinder interactions  
%     .dist - [time x num_cylinders x num_agents] distances to cylinder surfaces
%     .coll - [time x num_agents] logical, true if agent has cylinder collision
%     .min - [num_agents x num_cylinders] minimum distance each agent achieved to each cylinder
%   agent_sph - Agent-sphere interactions
%     .dist - [time x num_spheres x num_agents] distances to sphere surfaces  
%     .coll - [time x num_agents] logical, true if agent has sphere collision
%     .min - [num_agents x num_spheres] minimum distance each agent achieved to each sphere
%
% Note: Obstacle distances are center-to-surface. Negative values indicate agent is inside obstacle.
%       Collisions use <= threshold: agent-agent uses r_i+r_j, environment uses r_i only.

% Get time steps
num_timesteps = size(pos_ned, 1);

% Get number of agents
num_agents = size(pos_ned, 2) / 3;

% Handle r_coll input - scalar or vector
if isscalar(r_coll)
    r_coll_vec = r_coll * ones(num_agents, 1);  % Same radius for all agents
else
    r_coll_vec = r_coll(:);  % Ensure column vector
    if length(r_coll_vec) ~= num_agents
        error('r_coll must be scalar or vector of length num_agents');
    end
end

% Initialize distance matrices
dist_agent_agent = zeros(num_timesteps, num_agents, num_agents);
dist_agent_wall = zeros(num_timesteps, 6, num_agents);

% Count total obstacles
num_cylinders = 0;
num_spheres = 0;
if isfield(map, 'n_cyl')
    num_cylinders = map.n_cyl;
end
if isfield(map, 'n_spheres')
    num_spheres = map.n_spheres;
end

% Initialize obstacle distance matrices
dist_agent_cyl = zeros(num_timesteps, num_cylinders, num_agents);
dist_agent_sph = zeros(num_timesteps, num_spheres, num_agents);

% Initialize collision detection matrices
collisions_agent = false(num_timesteps, num_agents);
collisions_wall = false(num_timesteps, num_agents);
collisions_cyl = false(num_timesteps, num_agents);
collisions_sph = false(num_timesteps, num_agents);

% Process each timestep
for t = 1:num_timesteps
    % Extract positions at time t
    pos_t = reshape(pos_ned(t, :), 3, num_agents)';  % [num_agents x 3]

    % Iterate through agents
    for i = 1:num_agents  % "current" agent
    
        % 1. Agent-to-agent distances
        for j = 1:num_agents
            if i ~= j
                dist_agent_agent(t, j, i) = norm(pos_t(i, :) - pos_t(j, :));
            else
                dist_agent_agent(t, j, i) = NaN;  % Self-distance is NaN
            end
        end
    
        % 2. Agent-to-wall distances
        % Wall indices: 1=North_min, 2=North_max, 3=East_min, 4=East_max, 
        % 5=Down_min, 6=Down_max
        dist_agent_wall(t, 1, i) = abs(pos_t(i, 1) - map.arena_north(1));  % North min wall
        dist_agent_wall(t, 2, i) = abs(pos_t(i, 1) - map.arena_north(2));  % North max wall
        dist_agent_wall(t, 3, i) = abs(pos_t(i, 2) - map.arena_east(1));    % East min wall
        dist_agent_wall(t, 4, i) = abs(pos_t(i, 2) - map.arena_east(2));    % East max wall
        dist_agent_wall(t, 5, i) = abs(pos_t(i, 3) - map.arena_down(1));    % Down min wall
        dist_agent_wall(t, 6, i) = abs(pos_t(i, 3) - map.arena_down(2));    % Down max wall
    
        % 3. Agent to cylindrical obstacles distances
        if num_cylinders > 0
            for c = 1:num_cylinders
                cyl_center_north = map.buildings_north(c);
                cyl_center_east = map.buildings_east(c);
                cyl_radius = map.buildings_width(c) / 2;
                
                % Distance to cylinder surface (2D in horizontal plane)
                center_dist = sqrt((pos_t(i, 1) - cyl_center_north)^2 + ...
                                   (pos_t(i, 2) - cyl_center_east)^2);
                
                % Distance to surface (negative if inside)
                dist_agent_cyl(t, c, i) = center_dist - cyl_radius;
            end
        end

        % 4. Agent to spherical obstacles distances
        if num_spheres > 0
            for s = 1:num_spheres
                sph_center_north = map.spheres_north(s);
                sph_center_east = map.spheres_east(s);
                sph_center_down = map.spheres_down(s);
                sph_radius = map.spheres_r(s);

                % Distance to sphere surface (3D)
                center_dist = sqrt((pos_t(i, 1) - sph_center_north)^2 + ...
                    (pos_t(i, 2) - sph_center_east)^2 + ...
                    (pos_t(i, 3) - sph_center_down)^2);

                % Distance to surface (negative if inside)
                dist_agent_sph(t, s, i) = center_dist - sph_radius;
            end
        end


        % Collision detection for this timestep
        r_i = r_coll_vec(i);  % Current agent's collision radius

        % Agent-agent collisions (use sum of both agents' radii)
        for j = 1:num_agents
            if j ~= i
                r_j = r_coll_vec(j);
                if dist_agent_agent(t, j, i) <= (r_i + r_j)
                    collisions_agent(t, i) = true;
                    break;  % Found a collision, no need to check other agents
                end
            end
        end
        
        % Wall collisions (agent radius only)
        if any(dist_agent_wall(t, :, i) <= r_i)
            collisions_wall(t, i) = true;
        end
        
        % Cylinder collisions (agent radius only) 
        if num_cylinders > 0
            if any(dist_agent_cyl(t, :, i) <= r_i)
                collisions_cyl(t, i) = true;
            end
        end
        
        % Sphere collisions (agent radius only)
        if num_spheres > 0
            if any(dist_agent_sph(t, :, i) <= r_i)
                collisions_sph(t, i) = true;
            end
        end

    end  % loop through "current" agent

end  % loop through timesteps

% Calculate minimum distances across all timesteps
% Agent-agent: min over time dimension (1), then permute to get [num_agents x num_agents]
min_agent_agent = squeeze(min(dist_agent_agent, [], 1))';  % Result: [num_agents x num_agents]

% Agent-wall: min over time dimension (1), then permute to get [num_agents x 6]  
min_agent_wall = squeeze(min(dist_agent_wall, [], 1))';    % Result: [num_agents x 6]

% Agent-cylinder: min over time dimension (1), then permute to get [num_agents x num_cylinders]
if num_cylinders > 0
    min_agent_cyl = squeeze(min(dist_agent_cyl, [], 1))';  % Result: [num_agents x num_cylinders]
else
    min_agent_cyl = [];  % Empty matrix with proper dimensions
end

% Agent-sphere: min over time dimension (1), then permute to get [num_agents x num_spheres]
if num_spheres > 0
    min_agent_sph = squeeze(min(dist_agent_sph, [], 1))';  % Result: [num_agents x num_spheres]
else
    min_agent_sph = [];  % Empty matrix with proper dimensions
end

% Organize outputs
agent_agent.dist = dist_agent_agent;
agent_agent.coll = collisions_agent;
agent_agent.min = min_agent_agent;

agent_wall.dist = dist_agent_wall;
agent_wall.coll = collisions_wall;
agent_wall.min = min_agent_wall;

agent_cyl.dist = dist_agent_cyl;
agent_cyl.coll = collisions_cyl;
agent_cyl.min = min_agent_cyl;

agent_sph.dist = dist_agent_sph;
agent_sph.coll = collisions_sph;
agent_sph.min = min_agent_sph;

end