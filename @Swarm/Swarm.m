classdef Swarm < handle
    % SWARM - This class represents an ensemble of dynamic agents of type
    % "Drone"
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Swarm general properties:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % drones:
    %           vector of Drone objects
    % nb_agents:
    %           size of the above vector
    % equivalent_drone:
    %           for path planner, drone at the barycenter ...
    %           of the swarm for command computations
    % pos_ned:

    properties
        drones % a vector of Drone objects
        nb_agents % size of the above vector
        equivalent_drone % for path planner, drone at the barycenter ...
                         % of the swarm for command computations
        algorithm SwarmAlgorithm
        collisions_history
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = Swarm()
            self.drones = [];
            self.nb_agents = 0;
            self.collisions_history = [];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add_drone(self, drone_type, p_drone, p_battery, p_sim, p_physics, map, p_swarm)
            % Varargin is used for backwards-compatibility with original
            % code.
            
            % Track the number of drones in the swarm
            self.nb_agents = self.nb_agents + 1;

            % Initialize object
            drone = Drone(drone_type, p_drone, p_battery, p_sim, p_physics, map);
            
            % Swarm config: show that drone should be in swarm calcs
            drone.swarm_in_calcs = true;

            % Swarm config: copy over swarm control parameters
            if p_swarm.is_active_migration == true
                drone.swarm_control = 'migration';
                drone.v_ref = p_swarm.v_ref;
                drone.u_ref = p_swarm.u_ref;
            elseif p_swarm.is_active_goal == true
                drone.swarm_control = 'goal';
                drone.v_ref = p_swarm.v_ref;
                drone.x_goal = p_swarm.x_goal;
            end
            if isfield(p_swarm, 'r')
                % If defined for swarm, overwrite each agent's default
                drone.r_aware_agents = p_swarm.r;  
                drone.r_aware_obs = p_swarm.r;
            end

            % Add drone to list of drones in swarm
            self.drones = [self.drones; drone];     
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function add_n_drones(self, drone, n)
            for i = 1:n
                self.add_drone(drone);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function init_rand_pos(self, map_size)

            for i = 1:self.nb_agents
                self.drones(i).init_rand_pos(map_size);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos(self, pos)

            for i = 1:self.nb_agents
                self.drones(i).set_pos(pos(:, i));
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel(self, vel)

            for i = 1:self.nb_drones
                self.drones(i).set_vel(vel(:, i));
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_collision_history(self, collision_states)
            % UPDATE_COLLISION_HISTORY: Update collision states and history for all drones
            for i = 1:self.nb_agents
                self.drones(i).update_collision_history(collision_states(i));
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Pos_ned = get_pos_ned(self)
            % Return positions of the agent of the swarm in a matrix shape
            % of size 3 x nb_agents
            %
            %        agent_1   agent_2   ...   agent_N
            %   pn
            %   pe
            %   pd

            Pos_ned = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);
                Pos_ned(:, i) = drone.pos_ned;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Vel_ned = get_vel_ned(self)
            % Return velocities of the agents of the swarm in a matrix shape
            % of size 3 x nb_agents
            %        agent_1   agent_2   ...   agent_N
            %   vn
            %   ve
            %   vd
            
            Vel_ned = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);
                Vel_ned(:, i) = drone.vel_ned;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_state(self, state)
            Pos_ned = state(repmat([true true true false false false], ...
                self.nb_drones,1));
            Pos_ned = reshape(Pos_ned,3,[]);
            Vel_xyz = state(repmat([false false false true true true], ...
                self.nb_drones,1));
            Vel_xyz = reshape(Vel_xyz,3,[]);
            
            self.set_pos(Pos_ned);
            self.set_vel(Vel_xyz);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function state = get_state(self)
            Pos_ned = self.get_pos_ned();
            Vel_ned = self.get_vel_ned();
            state = [Pos_ned; Vel_ned];
            state = state(:);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Path_len = get_path_len(self)

            Path_len = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                drone = self.drones(i);

                Path_len(1, i) = drone.path_len;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel_commands(self, commands)

            for i = 1:self.nb_agents
                drone = self.drones(i);
                drone.prev_command = drone.command;
                drone.command(1) = 0;
                drone.command(2:4) = commands(:, i);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Qt = get_Qt(self)
            Qt = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                Qt(i) = self.drones(i).Qt;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Q = get_Q(self)
            Q = zeros(1, self.nb_agents);

            for i = 1:self.nb_agents
                Q(i) = self.drones(i).Q;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function colors = get_colors(self)
            colors = zeros(3, self.nb_agents);

            for i = 1:self.nb_agents
                colors(:, i) = self.drones(i).color;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_state(self, wind, time)

            for i = 1:self.nb_agents
                self.drones(i).update_state(wind, time);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [vel_commands, collisions] = update_command(self, p_swarm, r_coll, dt)

            switch lower(self.algorithm)
                case "vasarhelyi"
                    [vel_commands, collisions] = self.compute_vel_vasarhelyi(p_swarm, r_coll, dt);
                case "olfati_saber"
                    [vel_commands, collisions] = self.compute_vel_olfati_saber(p_swarm, r_coll, dt);
                case "vasarhelyi_original"
                    [vel_commands, collisions] = self.compute_vel_vasarhelyi_original(p_swarm, r_coll, dt);
                otherwise
                    error('Requested swarm algorithm has not been added to @Swarm: %s', self.algorithm)
            end

            % Save velocity commands to drones
            self.set_vel_commands(vel_commands);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_planner_swarm(self, path_type, time)
            % Creates an equivalent drone which will receive swarm
            % commands
            self.equivalent_drone = get_barycenter(self);
            self.equivalent_drone.plan_path(path_type, time);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function equivalent_drone = get_barycenter(self)
            pos = zeros(3, 1);
            vel = zeros(3, 1);

            for i = 1:self.nb_agents
                pos = pos + self.drones(i).pos_ned;
                vel = vel + self.drones(i).vel_xyz;
            end

            pos = pos / self.nb_agents;
            vel = vel / self.nb_agents;
            equivalent_drone = Drone(self.drones(1).drone_type, ...
                self.drones(1).p_drone, self.drones(1).p_battery, ...
                self.drones(1).p_sim, self.drones(1).p_physics, ...
                self.drones(1).map);
            equivalent_drone.set_pos(pos);
            equivalent_drone.set_vel(vel);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_planner_individual(self, path_type, time)
            % Each agent creates its waypoints independently
            for i = 1:self.nb_agents
                self.drones(i).plan_path(path_type, time);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_manager_individual(self, time)
            % Each agent creates its path independently
            for i = 1:self.nb_agents
                self.drones(i).path_manager_wing(time);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function path_follower_individual(self, time)
            % Each agent follows its path independently
            for i = 1:self.nb_agents
                self.drones(i).follow_path(time);
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function pos_ned_history = get_pos_ned_history(self)
            for i = 1:self.nb_agents
                pos_ned_history(:, (3 * (i - 1) + 1) : (3 * (i - 1) + 3)) = self.drones(i).pos_ned_history;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function vel_xyz_history = get_vel_xyz_history(self)
            vel_xyz_history = [];
            for i = 1:self.nb_agents
                vel_xyz_history(:, (3 * (i - 1) + 1) : (3 * (i - 1) + 3)) = self.drones(i).vel_xyz_history;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function vel_ned_history = get_vel_ned_history(self)
            vel_ned_history = [];
            for i = 1:self.nb_agents
                vel_ned_history(:, (3 * (i - 1) + 1) : (3 * (i - 1) + 3)) = self.drones(i).vel_ned_history;
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function collision_history = get_collision_history(self)
            collision_history = [];
            for i = self.nb_agents:-1:1
                collision_history(:, i) = self.drones(i).collision_history;
            end
            collision_history = logical(collision_history);

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function in_sim_history = get_in_sim_history(self)
            in_sim_history = [];
            for i = self.nb_agents:-1:1
                in_sim_history(:, i) = self.drones(i).in_sim_history;
            end
            in_sim_history = logical(in_sim_history);

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function bool_vec = get_swarm_in_calcs(self)
            % Return bool vector of whether agents should be included in
            % swarm calculations (size 1 x nb_agents)

            bool_vec = false(1, self.nb_agents);

            for i = 1:self.nb_agents
                bool_vec(i) = self.drones(i).swarm_in_calcs;
            end

        end

        function bool_vec = get_swarm_in_failsafe(self)
            % Return bool vector of whether agents are currently in
            % failsafe mode

            bool_vec = false(1, self.nb_agents);

            for i = 1:self.nb_agents
                bool_vec(i) = self.drones(i).state_failsafe;
            end

        end

        function bool_vec = get_swarm_in_sim(self)
            % Return bool vector of whether agents are designated as being
            % "in the simulation" (true = currently in the simulation)

            bool_vec = false(1, self.nb_agents);

            for i = 1:self.nb_agents
                bool_vec(i) = self.drones(i).state_in_sim;
            end

        end

        function bool_vec = get_swarm_in_collision(self)
            % Return bool vector of whether agents are designated as being
            % "in the simulation" (true = currently in the simulation)

            bool_vec = false(1, self.nb_agents);

            for i = 1:self.nb_agents
                bool_vec(i) = self.drones(i).collision;
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % // TODO: add this function to the SwarmViewer
        fig_handle = draw_agents_energy(self, time, period, fig_handle, axes_lim);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        record_state(self, time, T, period, is_active_cyl, ...
            collisions, map, dirname);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        record_video(self, time, T, period, fig_handle, path);
    end

end
 