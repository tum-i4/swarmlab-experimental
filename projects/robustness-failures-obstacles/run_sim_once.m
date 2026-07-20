function [pos_ned_history, vel_ned_history, time_history, ...
    p_swarm, map, collision_history, in_sim_history] = run_sim_once(...
    end_time, end_x, ...
    swarm_config, map, agent_starts, p_vasar, fail_config, ...
    results_dir, flags)
% Description of function


%%%%% Specify simulation options

% Simulation options

% DRONE_TYPE = "point_mass"; % swarming mode supports only quadcopter and point_mass
DRONE_TYPE = "quadcopter";
SWARM_ALGORITHM = "vasarhelyi";

% Set flags, save
flags.DRONE_TYPE = DRONE_TYPE;

% Tolerance for checking failure time (consider time step size of 0.01)
tol_time = 0.001;

% Set flags for obstacle types depending on map settings
if map.n_cyl>0
    flags.ACTIVE_OBSTACLES_CYLINDERS = true;
else
    flags.ACTIVE_OBSTACLES_CYLINDERS = false;
end
if map.n_spheres>0
    flags.ACTIVE_OBSTACLES_SPHERES = true; 
else
    flags.ACTIVE_OBSTACLES_SPHERES = false;
end

% Flags to activate obstacle types (used in param files)
ACTIVE_ENVIRONMENT = true; % activates all environment configs
ACTIVE_OBSTACLES_CYLINDERS = flags.ACTIVE_OBSTACLES_CYLINDERS;
ACTIVE_OBSTACLES_SPHERES = flags.ACTIVE_OBSTACLES_SPHERES;
ACTIVE_ARENA_WALLS = flags.ACTIVE_ARENA_WALLS;

CENTER_VIEW_ON_SWARM = false; % Only used for video


%%%%% Input checks

% Input check: sizes of failure agent and fail time vectors should be same
if ~isequal(length(fail_config.fail_agent_num), ...
        length(fail_config.fail_time))
    error('Sizes of fail_agent_num and fail_time do not match.')
end

% Input check: if more than one agent specified for failure, then vectors
% must not contain NaNs
num_agent_fail = length(fail_config.fail_agent_num);
if (num_agent_fail > 1) && ...
        ((num_agent_fail ~= sum(~isnan(fail_config.fail_agent_num))) || ...
        (num_agent_fail ~= sum(~isnan(fail_config.fail_time))))
    error('Multiple agents indicated for failure, but one or more agents or failure times were specified with NaN.')
end

% Input check: if failure agent or failure time is greater than number of
% agents or end time
for check_i = 1:num_agent_fail
    if ~isnan(fail_config.fail_agent_num(check_i)) && ...
            (fail_config.fail_agent_num(check_i) > swarm_config.num_agents)
        error('Failure agent is greater than total number of agents.')
    end
    if ~isnan(fail_config.fail_time(check_i)) && ...
            (fail_config.fail_time(check_i) > end_time)
        error('Failure time is later than sim end time.')
    end
end

% Check number of unique failure agents
if any(isnan(fail_config.fail_agent_num)) && ...
        isequal(length(fail_config.fail_agent_num), 1)
    num_unique_agent_fail = 0;
elseif any(isnan(fail_config.fail_agent_num)) && ...
        (length(fail_config.fail_agent_num) > 1)
    error('Check input specification of failure agents; at least one may be specified as non-failing.')
else
    num_unique_agent_fail = length(unique(fail_config.fail_agent_num));
end


%%%%% Generate parameter variables
end_time = round(end_time, 2);  % to handle machine error
p_sim = generate_p_sim(end_time);
run('param_battery');  % use default script
run('param_physics');  % use default script
run('param_drone');    % must use default script - it calls other scripts
p_swarm = generate_p_swarm(flags, swarm_config, p_vasar, map, agent_starts);
run('param_vasarhelyi')


%%%%% Adjust Vasarhelyi settings

if ~isempty(p_vasar)

    % Repulsion - only affects agent-agent interactions

    % Repulsion range
    p_swarm.r0_rep = p_swarm.d_ref; % radius of repulsion set in swarm config
    % Repulsion gain
    p_swarm.p_rep = p_vasar.p_rep;

    % Friction

    % Stopping point offset of alignment
    p_swarm.r0_fric = p_vasar.r0_fric;
    % Coefficient of velocity alignment
    p_swarm.C_fric = p_vasar.C_fric;
    % Velocity slack of alignement
    p_swarm.v_fric = p_vasar.v_fric;
    % Gain of braking curve
    p_swarm.p_fric = p_vasar.p_fric;
    % Acceleration of braking curve
    p_swarm.a_fric = p_vasar.a_fric;

    % Obstacles and wall parameters

    % Stopping point offset of walls
    p_swarm.r0_shill = p_vasar.r0_shill;
    % Velocity of virtual shill agents
    p_swarm.v_shill = p_vasar.v_shill;
    % Gain of bracking curve for walls
    p_swarm.p_shill = p_vasar.p_shill;
    % Acceleration of braking curve for walls
    p_swarm.a_shill = p_vasar.a_shill;

    % New parameters
    if isfield(p_vasar, 'r0_shill_arena')
        p_swarm.r0_shill_arena = p_vasar.r0_shill_arena;
    else
        p_swarm.r0_shill_arena = p_swarm.r0_shill;
    end
    if isfield(p_vasar, 'v_shill_arena')
        p_swarm.v_shill_arena = p_vasar.v_shill_arena;
    else
        p_swarm.v_shill_arena = p_swarm.v_shill;
    end
    if isfield(p_vasar, 'p_shill_arena')
        p_swarm.p_shill_arena = p_vasar.p_shill_arena;
    else
        p_swarm.p_shill_arena = p_swarm.p_shill;
    end
    if isfield(p_vasar, 'a_shill_arena')
        p_swarm.a_shill_arena = p_vasar.a_shill_arena;
    else 
        p_swarm.a_shill_arena = p_swarm.a_shill;
    end
end


%%%%% Check config parameters and set defaults if needed
[p_sim, p_battery, p_physics, p_drone, map, p_swarm] = ...
    check_params(p_sim, p_battery, p_physics, p_drone, map, p_swarm);


%%%%% Init Swarm Object, Wind, Viewer, and other variables

% Init swarm and set positions
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;

for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
        map, p_swarm);

    % Experiment 0 - no cooperation
    if isfield(flags, 'cooperating') && ~flags.cooperating
        swarm.drones(i).swarm_in_calcs = false;
    end

end
swarm.set_pos(p_swarm.Pos0);

% Init wind
wind = zeros(6,1); % steady wind (1:3), wind gusts (3:6)


%%%%% Main simulation loop

% Precalculate indices of non-failing agents
logi_no_fail_agent = ~ismember((1:swarm_config.num_agents), fail_config.fail_agent_num);

% Initialize counter for less-frequent command updates
command_dt = p_sim.dt_command;  % [sec] command update period
last_command_time = -command_dt;

% Sim Loop
for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Apply failure at failure time
    swarm = failure_manager(swarm, p_drone, time, fail_config, tol_time);

    % Compute velocity commands from swarming algorithm
    if (time - last_command_time) >= (command_dt - tol_time)
        [~,~] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt_command);
        last_command_time = time;
    end

    % Update swarm states - must be every time step for correct PID calcs
    swarm.update_state(wind, time);

    % Check for collisions and update agent collision and in_sim states if 
    % collisions occur.
    swarm = collision_manager(swarm, p_swarm, map, p_drone);

    % End sim if all agents cross end_x OR all agents are out of sim
    if flags.dynamic_end
        % Current x positions of all agents
        current_x = [swarm.drones.pos_ned];
        current_x = current_x(1, :);
        % current_x = current_x(1, logi_no_fail_agent);  % use if only focusing on non-failing agents
        % if ~isequal(length(current_x), swarm_config.num_agents-num_unique_agent_fail)  % Quick verify
        %     error('Length of x-positions does not match number of agents.')
        % end

        % Current in_sim states of all agents
        current_in_sim = [swarm.drones.state_in_sim];

        % Check and break if applicable
        if all(current_x > end_x) || ...  % all across
                all(~current_in_sim) || ...  % none left in sim
                ((sum(current_in_sim) > 0) && ...  
                    all(current_x(current_in_sim) > end_x))  % all remaining in sim are across
            break
        end
    end

end


%%%%% Analyse states and save

% time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
time_history = p_sim.start_time:p_sim.dt:time;
pos_ned_history = swarm.get_pos_ned_history();
pos_ned_history = pos_ned_history(2:end,:);
vel_ned_history = swarm.get_vel_ned_history();
accel_history = [zeros(1, p_swarm.nb_agents*3); ...
    diff(vel_ned_history,1)/p_sim.dt];
collision_history = swarm.get_collision_history();
swarm.collisions_history = collision_history;
in_sim_history = swarm.get_in_sim_history();

% Save workspace
if ~flags.run_parallel && flags.save
    workspace_path = strcat(results_dir,'/state_var');
    save(workspace_path,'time_history','pos_ned_history','vel_ned_history', ...
        'accel_history', 'collision_history', 'in_sim_history', ...
        'p_sim', 'p_swarm', 'map');
end


%%%%% Run visualization and postprocessing
if ~flags.run_parallel && flags.viz_video

    % This is a cool visualization tool of theirs, but I don't think it saves anything
    SwarmViewerOffline(p_sim.dt_video, ...
        CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map, p_swarm);

end

if ~flags.run_parallel && flags.viz

    % Plot state variables
    fontsize = 12;
    agents_color = swarm.get_colors();
    lines_color = [];

    % Save if flagged
    if flags.save
        plot_results_dir = results_dir;
    else
        plot_results_dir = [];
    end
    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        plot_results_dir);

end

end % end main function


%% Local functions


% p_sim
function p_sim = generate_p_sim(end_time)

p_sim.dt = 0.01; % simulation dt [s]  % 0.01 required for quadcopter model
p_sim.dt_command = 0.1; % [s] refresh period for swarm control calculation
p_sim.dt_plot = 0.5; % period of plots update [s]
p_sim.dt_video = 0.1; % period of video update [s]
p_sim.start_time = 0; % [s]
p_sim.end_time = end_time; % [s]

end


% p_swarm - IMPORTANT for specifying swarm's goal behavior
function p_swarm = generate_p_swarm(flags, swarm_config, p_vasar, map, starts)

% Variables to be set
p_swarm.is_active_migration = true;
p_swarm.is_active_goal = false;
p_swarm.is_active_arena = flags.ACTIVE_ARENA_WALLS;
p_swarm.is_active_spheres = flags.ACTIVE_OBSTACLES_SPHERES;
p_swarm.is_active_cyl = flags.ACTIVE_OBSTACLES_CYLINDERS;

% Set number of drones in swarm
p_swarm.nb_agents = swarm_config.num_agents; 

% Max radius of influence - neighborhood [m]
p_swarm.r = swarm_config.r;

% Max topological distance number of neighbors
p_swarm.max_neig = 10;

% Radius of collision
p_swarm.r_coll = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arena parameters - Cubic arena
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.x_arena = [map.arena_north; map.arena_east; map.arena_down];
p_swarm.center_arena = sum(p_swarm.x_arena, 2) / 2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spheric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.spheres = [map.spheres_north; map.spheres_east; ...
    map.spheres_down; map.spheres_r];

p_swarm.n_spheres = map.n_spheres;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylindric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cylinder_radius = map.buildings_width ./ 2;

p_swarm.cylinders = [
    map.buildings_north; % x_obstacle
    map.buildings_east; % y_obstacle
    cylinder_radius]; % r_obstacle
p_swarm.n_cyl = map.n_cyl;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference values
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inter-agent distance
p_swarm.d_ref = p_vasar.d_ref;

% Velocity direction
p_swarm.u_ref = [1 0 0]';

% Speed
p_swarm.v_ref = swarm_config.v_ref;

% Goal location
p_swarm.x_goal = [350, 100, -50]';
% p_swarm.x_goal = [950, 100, -50]';  % To test out steady-state interactions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity and acceleration bounds for the agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flags.DRONE_TYPE == "fixed_wing" || flags.DRONE_TYPE == "quadcopter"
    p_swarm.max_a = []; % leave empty if you use a real drone model
    p_swarm.max_a_vasar = 30; % bounds how much vasarhelyi commands can change
    % p_swarm.max_a_vasar = []; % no bound for possible changes in commands
elseif flags.DRONE_TYPE == "point_mass"
    p_swarm.max_a = 10;
else
	error('DRONE_TYPE is incorrectly specified.')
end
p_swarm.max_v = 15;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position and velocity for the swarm
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Use pre-defined initial positions so that results are reproducible
p_swarm.P0 = [-(starts.start_buffer + (starts.Lx/2)), ...
    (starts.Ly/2), -(starts.Lz/2)]';
p_swarm.Pos0 = p_swarm.P0 + starts.agent_positions';

% Assume starting velocity is zero
p_swarm.Vel0 = zeros(size(p_swarm.Pos0));

end


% Manage failure executions
function swarm = failure_manager(swarm, p_drone, time, ...
    fail_config, tol_time)

% Check if flag specifies type of failure, otherwise default to 0
if isfield(fail_config, "failure_option")
    flag_failure_option = round(fail_config.failure_option);
else
    flag_failure_option = 0;  % Use default failure (stop and hold)
end

% Logical - check for failures that should occur on this time step
fail_now = abs(time - fail_config.fail_time) < tol_time;  % tolerance to handle machine error

if any(fail_now)

    % Agent numbers that should fail on this time step
    vect_fail_now = fail_config.fail_agent_num(fail_now);

    % Iterate through agents that should fail at this time step
    for i = 1:length(vect_fail_now)
        
        switch flag_failure_option

            case 0
                % Failure mode: stop flight and hold position
                swarm.drones(vect_fail_now(i)).state_failsafe = true;
                swarm.drones(vect_fail_now(i)).swarm_control = 'velocity';
                swarm.drones(vect_fail_now(i)).u_ref = [0; 0; 0];
                swarm.drones(vect_fail_now(i)).v_ref = 0;

            case 1
                % Failure mode: max angular velocity for all motors reduced
                vect_fail_now_power = fail_config.fail_power(fail_now);  % get fail power
                vect_fail_now_power = vect_fail_now_power(i);

                % Apply change in max angular velocity
                swarm.drones(vect_fail_now(i)).p_drone.k_omega = ...
                    p_drone.k_omega * vect_fail_now_power;

            case 2
                % Failure mode: change in drone's direction and speed
                
                % Extract vector components of new direction
                new_v_n = fail_config.fail_v_n(fail_now);
                new_v_n = new_v_n(i);
                new_v_e = fail_config.fail_v_e(fail_now);
                new_v_e = new_v_e(i);
                new_v_d = fail_config.fail_v_d(fail_now);
                new_v_d = new_v_d(i);

                % Get magnitude of vector, set as new reference speed
                new_speed = norm([new_v_n, new_v_e, new_v_d]);
                swarm.drones(vect_fail_now(i)).v_ref = new_speed;

                % Set new reference direction (unit vector)
                swarm.drones(vect_fail_now(i)).swarm_control = 'velocity';
                swarm.drones(vect_fail_now(i)).u_ref = [
                    new_v_n / new_speed; ...
                    new_v_e / new_speed; ...
                    new_v_d / new_speed];

            otherwise
                error("Failure option incorrectly specified")

        end

        if ~fail_config.fail_in_swarm
            swarm.drones(vect_fail_now(i)).swarm_in_calcs = false;
        end

    end

end

end


% Manage collisions that occur during simulation
function swarm = collision_manager(swarm, p_swarm, map, p_drone)
% If collision occurs between agents or between an agent and the
% environment (obstacle or arena wall), the agent should freeze and be
% removed from the simulation.

    % First check who is still in the sim
    in_sim = swarm.get_swarm_in_sim;

    % Get current positions and reshape for calculate_all_distances
    positions_3xN = swarm.get_pos_ned();  % 3xN matrix

    % Filter out any agent not currently in the sim
    positions_3xN = positions_3xN(:, in_sim);

    % Number of agents still in sim
    nb_agents_in_sim = sum(in_sim);
    
    % Reshape positions to [1 x (3*num_agents)] format expected by utility
    pos_ned_row = reshape(positions_3xN, 1, 3*nb_agents_in_sim);
    
    % Use collision radius from swarm parameters
    r_coll = p_swarm.r_coll;
    
    % Calculate all distances and collision detections - ONLY for agents
    % that are in_sim
    [agent_agent, agent_wall, agent_cyl, agent_sph] = ...
        calculate_all_distances(pos_ned_row, map, r_coll);
    
    % Extract collision matrices (single timestep, so use t=1)
    t = 1;
    % agent-agent
    collisions_agent = false(size(in_sim));
    collisions_agent(in_sim) = agent_agent.coll(t, :);
    % agent-wall
    collisions_wall = false(size(in_sim));
    collisions_wall(in_sim) = agent_wall.coll(t, :);
    % agent-cyl
    collisions_cyl = false(size(in_sim));
    collisions_cyl(in_sim) = agent_cyl.coll(t, :);
    % agent-sph
    collisions_sph = false(size(in_sim));
    collisions_sph(in_sim) = agent_sph.coll(t, :);
    
    % Determine overall collision state for each agent still in sim
    collisions_now = collisions_agent | collisions_wall | collisions_cyl | collisions_sph;

    % Combine with collision state of agents not in sim and update
    % collisions states and history for all agents
    in_collision = swarm.get_swarm_in_collision;
    overall_collisions = collisions_now;
    overall_collisions(~in_sim) = in_collision(~in_sim);
    swarm.update_collision_history(overall_collisions);
    
    % Handle specific collision responses, update state_in_sim for each
    % agent.
    for i = 1:swarm.nb_agents
        % Handle agent-agent collisions.
        if collisions_agent(i)
            % Remove agent from simulation and freeze
            swarm.drones(i).update_in_sim_history(false);
            swarm.drones(i).swarm_control = 'velocity';
            swarm.drones(i).u_ref = [0; 0; 0];
            swarm.drones(i).v_ref = 0;
        
        % Handle environment collisions (wall, cylinder, sphere)
        elseif collisions_wall(i) || collisions_cyl(i) || collisions_sph(i)
            % Remove agent from simulation and freeze
            swarm.drones(i).update_in_sim_history(false);
            swarm.drones(i).swarm_control = 'velocity';
            swarm.drones(i).u_ref = [0; 0; 0];
            swarm.drones(i).v_ref = 0;
            % % Reset k_omega in case it was falling from a collision
            % swarm.drones(i).p_drone.k_omega = p_drone.k_omega;

        else

            % Make sure that the in_sim history is updated even if a
            % collision does not happen. This will be used in
            % drone.update_state method.
            if swarm.drones(i).state_in_sim == false
                swarm.drones(i).update_in_sim_history(false);
            else
                swarm.drones(i).update_in_sim_history(true);
            end

        end
    end

end