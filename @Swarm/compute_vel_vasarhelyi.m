function [vel_command, collisions] = compute_vel_vasarhelyi(self, p_swarm, r_agent, dt)
    % VASARHELYI SWARM ALGORITHM
    % This is an implementation of the Vasarhelyi algorithm. It allows the
    % navigation of a swarm of agents in presence of obstacles and walls.
    %
    % Ref:      Vasarhelyi, Science Robotics, 2018
    % 
    % Modif:    a cohesion term has been added to make the agents get
    %           closer when they are farther than r0_rep.
    %
    % Inputs:
    %   p_swarm: swarm parameters
    %   r_agent: safety radius of agents
    %   dt: time step
    %
    % Outputs:
    %   vel_command: commanded velocities for every agent
    %   collisions: [nb_agent_collisions nb_obs_collisions min_dist_obs]
    %

    % Initialize variables
    pos = self.get_pos_ned();           % [3xn] matrix of positions
    vel = self.get_vel_ned();           % [3xn] matrix of velocities
    nb_agents = self.nb_agents;         % n agents

    vel_command = zeros(3, nb_agents);  % Total commanded velocity
    vel_rep = zeros(3, nb_agents);      % Repulsion velocity
    vel_rep_only = zeros(3, nb_agents); % Repulsion velocity for repulsion-only interactions
    vel_fric = zeros(3, nb_agents);     % Velocity matching velocity
    vel_obs = zeros(3, nb_agents);      % Obstacle repulsion velocity
    vel_wall = zeros(3, nb_agents);     % Arena repulsion velocity
    vel_goal_obs = zeros(3, nb_agents); % Goal repulsion velocity
    
    nb_agent_collisions = 0;            % Number of collisions among agents
    nb_obs_collisions = 0;              % Number of collisions against obstacles
    min_dist_obs = inf;                 % (Unclear) track how close drones get to obstacles

    % Check which drones are in swarm calcs
    in_calcs = self.get_swarm_in_calcs();

    % Check which drones are in failsafe state (note: this does not
    % necessarily mean that the other drones will exclude this one from 
    % swarm calcs).
    % Failsafe currently defined to mean drone will ignore all interactions
    % except for goal.
    in_failsafe = self.get_swarm_in_failsafe();


    % Iterate through each agent and compute velocity command
    for agent = 1:nb_agents
        
        % If agent is calculating own swarm calcs
        if in_calcs(agent) && ~in_failsafe(agent)

            % Indicies of all OTHER agents that are also in swarm calcs
            others_in_calcs = in_calcs;
            others_in_calcs(agent) = false; % take self out of vector

            % Indicies of all OTHER agents NOT in swarm calcs
            others_out_calcs = ~others_in_calcs;
            others_out_calcs(agent) = false; % take self out of vector

            % Intra-Swarm Interactions
            [vel_rep(:, agent), vel_fric(:, agent), nb_agent_collisions] = compute_intra_swarm_interactions(...
                pos(:, agent), pos(:, others_in_calcs), vel(:, agent), vel(:, others_in_calcs), ...
                p_swarm, r_agent, nb_agent_collisions);

            % Drone-Environment Interactions
            [vel_obs(:, agent), vel_wall(:, agent), nb_obs_collisions, min_dist_obs] = compute_drone_environment_interactions(...
                pos(:, agent), vel(:, agent), p_swarm, r_agent, nb_obs_collisions, min_dist_obs);

            % Drone Repulsion-Only Interactions - any drones NOT in swarm
            [vel_rep_only(:, agent), nb_agent_collisions] = compute_repulsion_only_interactions(...
                pos(:, agent), pos(:, others_out_calcs), p_swarm, r_agent, nb_agent_collisions);

            % Active Goal repulsion to prevent convergence
            if p_swarm.is_active_goal == true
                [vel_goal_obs(:, agent)] = compute_active_goal_influence(...
                    pos(:, agent), vel(:, agent), p_swarm, p_swarm.x_goal);
            end

            % Sum agent-agent and obstacle contributions to velocity command
            vel_command(:, agent) = vel_rep(:, agent) + vel_fric(:, agent) + vel_obs(:, agent) + vel_wall(:, agent)...
                + vel_goal_obs(:, agent) + vel_rep_only(:, agent);

            % Add self propulsion OR migration term based on swarm config
            vel_command(:, agent) = compute_velocity_goal(vel_command(:, agent), pos(:, agent), vel(:, agent), p_swarm, ...
                self.drones(agent).v_ref, self.drones(agent).u_ref, self.drones(agent).x_goal);

        end
        
        % If agent is not in swarm calcs
        if ~in_calcs(agent) || in_failsafe(agent)

            % Easier access to drone properties
            goal_calc_type = self.drones(agent).swarm_control;  % How out-of-swarm drone should be controlled to own goal

            % Drone-Environment Interactions
            [vel_obs(:, agent), vel_wall(:, agent), nb_obs_collisions, min_dist_obs] = compute_drone_environment_interactions(...
                pos(:, agent), vel(:, agent), p_swarm, r_agent, nb_obs_collisions, min_dist_obs);

            % Drone Repulsion-Only Interactions (against all other drones)
            [vel_rep_only(:, agent), nb_agent_collisions] = compute_repulsion_only_interactions(...
                pos(:, agent), pos(:, 1:end ~= agent), p_swarm, r_agent, nb_agent_collisions);

            % ONLY IF agent is in failsafe mode: Skip interactions with 
            % (1) other drones and (2) with environment
            if in_failsafe(agent)
                vel_obs(:, agent) = zeros(size(vel_obs(:, agent)));
                vel_wall(:, agent) = zeros(size(vel_wall(:, agent)));
                vel_rep_only(:, agent) = zeros(size(vel_rep_only(:, agent)));
            end

            % Active Goal repulsion to prevent convergence
            if ismember(lower(goal_calc_type), {'goal', 'active goal', 'active_goal'})
                [vel_goal_obs(:, agent)] = compute_active_goal_influence(...
                        pos(:, agent), vel(:, agent), p_swarm, self.drones(agent).x_goal);
            end

            % Sum agent-agent and obstacle contributions to velocity command
            vel_command(:, agent) = vel_obs(:, agent) + vel_wall(:, agent)...
                + vel_goal_obs(:, agent) + vel_rep_only(:, agent);

            % Add velocity goal or active goal
            if ismember(lower(goal_calc_type), {'velocity', 'migration'})

                % Velocity migration
                temp_flags.is_active_migration = true;
                temp_flags.is_active_goal = false;
                vel_command(:, agent) = compute_velocity_goal(vel_command(:, agent), pos(:, agent), vel(:, agent), ...
                    temp_flags, self.drones(agent).v_ref, self.drones(agent).u_ref, self.drones(agent).x_goal);

            elseif ismember(lower(goal_calc_type), {'goal', 'goal', 'active_goal'})

                % Active goal
                temp_flags.is_active_migration = false;
                temp_flags.is_active_goal = true;
                vel_command(:, agent) = compute_velocity_goal(vel_command(:, agent), pos(:, agent), vel(:, agent), ...
                    temp_flags, self.drones(agent).v_ref, self.drones(agent).u_ref, self.drones(agent).x_goal);

            else

                error('Control method not correctly specified for drone leaving swarm calculations.')

            end
            

        end

    end

    % Total number of collisions per time step
    nb_agent_collisions = nb_agent_collisions / 2; % reciprocal
    collisions = [nb_agent_collisions nb_obs_collisions min_dist_obs];

    % Add random effect on velocities
    if isfield(p_swarm, 'c_r')
        vel_command = vel_command + p_swarm.c_r * randn(3, nb_agents);
    end

    % Bound velocities and acceleration
    if ~isempty(p_swarm.max_v)
        vel_cmd_norm = sqrt(sum((vel_command.^2), 1));
        
        idx_to_bound = (vel_cmd_norm > p_swarm.max_v);
        if sum(idx_to_bound) > 0
            vel_command(:, idx_to_bound) = p_swarm.max_v * ...
                vel_command(:, idx_to_bound) ./ repmat(vel_cmd_norm(idx_to_bound), 3, 1);
        end
    end
    if ~isempty(p_swarm.max_a)
        accel_cmd = (vel_command-vel)./dt;
        accel_cmd_norm = sqrt(sum(accel_cmd.^2, 1));
        idx_to_bound = ( accel_cmd_norm > p_swarm.max_a | accel_cmd_norm < - p_swarm.max_a);
        if sum(idx_to_bound) > 0
            vel_command(:, idx_to_bound) = vel(:, idx_to_bound) + ...
                dt*p_swarm.max_a * accel_cmd(:, idx_to_bound) ./ ...
                repmat(accel_cmd_norm(idx_to_bound), 3, 1);
        end
    end


end


%% Migration, Active Goal, or Self-Propulsion
function [vel_command] = compute_velocity_goal(vel_command, pos_self, vel_self, flags_source, v_ref, u_ref, x_goal)

    % Add self propulsion OR migration term based on swarm config
    v_norm = sqrt(sum((vel_self.^2), 1));

    if flags_source.is_active_migration == true % migration
        vel_command = vel_command + v_ref * u_ref;

    elseif flags_source.is_active_goal == true % goal
        x_goal_rel = x_goal - pos_self;
        u_goal = x_goal_rel / norm(x_goal_rel);
        vel_command = vel_command + v_ref * u_goal;

    elseif v_norm > 0 % self-propulsion when no swarm 
            vel_command = vel_command + v_ref * vel_self / v_norm;
    end

end


%% Active Goal Destination Influence
function [vel_goal_obs] = compute_active_goal_influence(...
    pos_self, vel_self, p_swarm, x_goal)
    % Run this is Active Goal is specified; adds a pseudo-obstacle at the
    % goal to prevent agents from converging and crashing together.
    % 
    % x_goal is a 3x1 vector with NED coordinates [m]

    % Initialize
    vel_goal_obs = zeros(3, 1);

    % Get obstacle center and radius
    c_obs = x_goal;
    r_obs = 1e-3; % [m] set artifically small radius

    % Compute distance agent(a)-obstacle(b)
    dist_ab = sqrt(sum((pos_self - c_obs).^2)) - r_obs;

    % Commented out because do NOT count as collision
    % nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

    % Set the virtual speed of the obstacle direction out of
    % the obstacle
    v_obs_virtual = (pos_self - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill;

    % Compute relative velocity agent-obstacle
    vel_ab = sqrt(sum((vel_self - v_obs_virtual).^2));

    v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

    if vel_ab > v_obs_max
        vel_goal_obs = vel_goal_obs + (vel_ab - v_obs_max) * (v_obs_virtual - vel_self) ./ vel_ab;
    end

end

%% Intra-Swarm Interactions
function [vel_rep, vel_fric, nb_agent_collisions] = compute_intra_swarm_interactions(...
    pos_self, pos_others, vel_self, vel_others, p_swarm, r_agent, nb_agent_collisions)

    % Initialize outputs - repulsion & friction
    vel_rep = zeros(3, 1);
    vel_fric = zeros(3, 1);

    % If no other drones, then end early
    if isempty(pos_others) || isempty(vel_others)
        return
    end
    
    % Compute agent-agent distance matrix
    p_rel = pos_others - pos_self;
    dist = sqrt(sum(p_rel.^2, 1));
    
    % Initialize neighbours list
    neig_list = find(dist ~= 0);
    
    % Count collisions
    nb_agent_collisions = nb_agent_collisions + sum(dist < (2 * r_agent));
    
    % Constraint on neighborhood given by the Euclidean distance
    if ~isempty(neig_list) && isfield(p_swarm, 'r')
        neig_list = neig_list(dist(neig_list) < p_swarm.r);
    end
    
    % Constraint on neighborhood given by the topological distance
    if ~isempty(neig_list) && isfield(p_swarm, 'max_neig') && ...
            (length(neig_list) > p_swarm.max_neig)
        [~, idx] = sort(dist(neig_list));
        neig_list = neig_list(idx(1:p_swarm.max_neig));
    end
    
    % Calculate repulsion, attraction, and velocity alignment
    if ~isempty(neig_list)

        % Compute velocity and position unit vectors between agent pairs
        v_rel = vel_others - vel_self;
        v_rel_norm = sqrt(sum(v_rel.^2, 1));
        p_rel_u = -p_rel ./ dist;
        v_rel_u = -v_rel ./ v_rel_norm;
    
        for agent2 = neig_list

            % Repulsion and attraction
            if dist(agent2) < p_swarm.r0_rep
                vel_rep = vel_rep + p_swarm.p_rep * (p_swarm.r0_rep - dist(agent2)) * p_rel_u(:, agent2);
            else
                vel_rep = vel_rep + p_swarm.p_rep * (dist(agent2) - p_swarm.r0_rep) * -p_rel_u(:, agent2);
            end
    
            % Velocity alignment
            v_fric_max = get_v_max(p_swarm.v_fric, dist(agent2) - p_swarm.r0_fric, p_swarm.a_fric, p_swarm.p_fric);
            if v_rel_norm(agent2) > v_fric_max
                vel_fric = vel_fric + p_swarm.C_fric * (v_rel_norm(agent2) - v_fric_max) * v_rel_u(:, agent2);
            end
        end

    end
end


%% Drone-Environment Interactions
function [vel_obs, vel_wall, nb_obs_collisions, min_dist_obs] = compute_drone_environment_interactions(...
    pos_self, vel_self, p_swarm, r_agent, nb_obs_collisions, min_dist_obs)

    % Initialize outputs - velocity contributions from obstacles, walls,
    % and goal (only if active_goal on)
    vel_obs = zeros(3, 1);
    vel_wall = zeros(3, 1);

    % Arena walls repulsion effects
    if (p_swarm.is_active_arena == true)
        unit = eye(3);
        %On each axis we have the two repulsions
        for axis = 1:3
            %On each axis there is two forces (each side of the arena)
            for dir = 1:2
                dist_ab = abs(pos_self(axis) - p_swarm.x_arena(axis, dir));

                %Compute velocity of wall shill agent toward center of the arena
                v_wall_virtual = unit(:, axis) .* p_swarm.v_shill;

                if dir == 2
                    v_wall_virtual = -v_wall_virtual;
                end

                %Compute relative velocity (Wall - Agent)
                vel_ab = sqrt(sum((vel_self - v_wall_virtual).^2));

                v_wall_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

                if vel_ab > v_wall_max
                    vel_wall = vel_wall + (vel_ab - v_wall_max) * (v_wall_virtual - vel_self) ./ vel_ab;
                end
            end
        end
    end

    % Compute spherical obstacles effects
    if (p_swarm.is_active_spheres == true)

        for obs = 1:p_swarm.n_spheres
            % Get obstacle center and radius
            c_obs = p_swarm.spheres(1:3, obs);
            r_obs = p_swarm.spheres(4, obs);

            % Compute distance agent(a)-obstacle(b)
            dist_ab = sqrt(sum((pos_self - c_obs).^2)) - r_obs;
            nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

            % Set the virtual speed of the obstacle direction out of
            % the obstacle
            v_obs_virtual = (pos_self - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill;

            % Compute relative velocity agent-obstacle
            vel_ab = sqrt(sum((vel_self - v_obs_virtual).^2));

            if dist_ab < min_dist_obs
                min_dist_obs = dist_ab;
            end

            v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

            if vel_ab > v_obs_max
                vel_obs = vel_obs + (vel_ab - v_obs_max) * (v_obs_virtual - vel_self) ./ vel_ab;
            end
        end
    end

    % Compute cylindrical obstacles effects
    if (p_swarm.is_active_cyl == true)

        for obs = 1:p_swarm.n_cyl
            % Get obstacle center and radius
            c_obs = p_swarm.cylinders(1:2, obs);
            r_obs = p_swarm.cylinders(3, obs);

            % Compute distance agent(a)-obstacle(b)
            dist_ab = sqrt(sum((pos_self(1:2) - c_obs).^2)) - r_obs;
            nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

            % Set the virtual speed of the obstacle direction out of
            % the obstacle
            v_obs_virtual = (pos_self(1:2) - c_obs) / (dist_ab + r_obs) * p_swarm.v_shill;

            % Compute relative velocity agent-obstacle
            vel_ab = sqrt(sum((vel_self(1:2) - v_obs_virtual).^2));

            if dist_ab < min_dist_obs
                min_dist_obs = dist_ab;
            end

            v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

            if vel_ab > v_obs_max
                vel_obs(1:2) = vel_obs(1:2) + (vel_ab - v_obs_max) * (v_obs_virtual - vel_self(1:2)) ./ vel_ab;
            end
        end
    end

    % Compute block obstacles effects
    if (p_swarm.is_active_blocks == true)

        for obs = 1:size(p_swarm.blocks_limits, 2)

            % First need to check if drone is inside block (collision)
            [in_block, dist_check, u_block] = obstacles.check_inside_block(...
                [pos_self(1), pos_self(2), pos_self(3)], ...
                p_swarm.blocks_limits(1:2, obs), ...
                p_swarm.blocks_limits(3:4, obs), ...
                p_swarm.blocks_limits(5:6, obs));

            if in_block

                % Drone is inside block, no need to check further
                nb_obs_collisions = nb_obs_collisions + 1;

                if dist_check < min_dist_obs
                    min_dist_obs = dist_check;
                end

                continue
                
            end

            % Check how many sides the drone can see (use dot product)
            num_sides_in_view = 0;
            sides_in_view = [];
            for j = 1:6 % iterate through the six sides
                if dot(p_swarm.blocks_normals{obs}(j, :), pos_self' - p_swarm.blocks_centers{obs}(j, :)) > 0
                    num_sides_in_view = num_sides_in_view + 1;
                    sides_in_view(end+1) = j;
                end
            end

            if num_sides_in_view == 1  % If only one face can be seen
                
                % % Get normal vector of the face that can be seen and
                % distance between face and agent.
                ind_face_start = (sides_in_view(1)-1) * 3 + 1;
                [relative_unit_vector, dist_ab] = obstacles.get_single_face_influence_vect(...
                    pos_self', vel_self', ...
                    p_swarm.blocks_faces{obs}(:, ind_face_start:ind_face_start+2), ...
                    p_swarm.blocks_normals{obs}(sides_in_view, :), ...
                    p_swarm.blocks_centers{obs}(sides_in_view, :));
                relative_unit_vector = relative_unit_vector';

            elseif num_sides_in_view == 2 || num_sides_in_view == 3

                % Use normal vector and distance to closest point on the
                % block
                relative_unit_vector = -1.*u_block';
                dist_ab = dist_check;

            else

                error('Did not account for this condition.')

            end

            % Check distance for collision
            nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

            % Set the virtual speed of the obstacle direction out of
            % the obstacle
            v_obs_virtual = relative_unit_vector * p_swarm.v_shill;

            % Compute relative velocity agent-obstacle
            vel_ab = sqrt(sum((vel_self - v_obs_virtual).^2));

            if dist_ab < min_dist_obs
                min_dist_obs = dist_ab;
            end

            v_obs_max = get_v_max(0, dist_ab - p_swarm.r0_shill, p_swarm.a_shill, p_swarm.p_shill);

            if vel_ab > v_obs_max
                vel_obs = vel_obs + (vel_ab - v_obs_max) * (v_obs_virtual - vel_self) ./ vel_ab;
            end

        end

    end

    

end


%% Drone Repulsion-Only Interactions
function [vel_rep, nb_agent_collisions] = compute_repulsion_only_interactions(...
    pos_self, pos_others, p_swarm, r_agent, nb_agent_collisions)
    % From intra-swarm interactions but only the repulsion part

    % Initialize output - repulsion only
    vel_rep = zeros(3, 1);

    % If no other drones, then end early
    if isempty(pos_others)
        return
    end
    
    % Compute agent-agent distance matrix
    p_rel = pos_others - pos_self;
    dist_to_others = sqrt(sum(p_rel.^2, 1));
    
    % Count collisions
    nb_agent_collisions = nb_agent_collisions + sum(dist_to_others < (2 * r_agent));

    % Compute position unit vectors between agent pairs
    p_rel_u = -p_rel ./ dist_to_others;

    for agent2 = 1:length(dist_to_others)

        % Repulsion only within repulsion radius, otherwise ignore other
        % drone
        if dist_to_others(agent2) < p_swarm.r0_rep
            vel_rep = vel_rep + p_swarm.p_rep * (p_swarm.r0_rep - dist_to_others(agent2)) * p_rel_u(:, agent2);
        end

    end

end

%% Calculate V fric max

function [ v_fricmax ] = get_v_max(v_fric, r, a, p)

    if r < 0
        v_fricmax = 0;
    elseif r * p > 0 && r * p < a / p
        v_fricmax = r * p;
    else
        v_fricmax = sqrt(2 * a * r - a^2 / p^2);
    end

    if v_fricmax < v_fric
        v_fricmax = v_fric;
    end
end
