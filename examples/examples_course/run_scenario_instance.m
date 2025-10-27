function [pos_ned_history, vel_ned_history, time_history, ...
    p_swarm, map, collision_history, in_sim_history] = run_scenario_instance( ...
    end_time, end_x, ...
    swarm_config, fail_config, ...
    results_dir, flags)
% RUN_SCENARIO_INSTANCE Execute a single UAV swarm simulation scenario
%
% This function runs a single instance of a UAV swarm simulation with specified
% parameters, including swarm algorithm, environment obstacles, and optional
% agent failures. The simulation uses quadcopter dynamics and collision detection.
%
% SYNTAX:
%   [pos_ned_history, vel_ned_history, time_history, p_swarm, map, ...
%    collision_history, in_sim_history] = run_scenario_instance(end_time, ...
%    end_x, swarm_config, fail_config, results_dir, flags)
%
% INPUTS:
%   end_time        - (double) Maximum simulation duration [s]
%
%   end_x           - (double) X-coordinate [m] defining end of scenario
%                     Used with flags.dynamic_end to stop simulation early
%
%   swarm_config    - (struct) Swarm configuration
%                     Fields:
%                       .num_agents  - Number of agents in swarm
%                       .swarm_algo  - Algorithm: "vasarhelyi", "vasarhelyi_original",
%                                      "olfati_saber", or "course_controller"
%
%   fail_config     - (struct) Failure scenario configuration
%                     Fields:
%                       .failure_option - Failure type: 1 (power cutoff) or
%                                         2 (velocity change)
%                       .fail_agent_num - Vector of agent indices to fail
%                       .fail_time      - Vector of failure times [s]
%                       .obs_n          - Obstacle N-coordinate [m]
%                       .obs_e          - Obstacle E-coordinate [m]
%                     For failure_option = 2, also include:
%                       .fail_v_n       - New velocity north component [m/s]
%                       .fail_v_e       - New velocity east component [m/s]
%                       .fail_v_d       - New velocity down component [m/s]
%
%   results_dir     - (string) Directory path for saving results
%                     If empty, results are not saved to individual files
%
%   flags           - (struct) Simulation control flags
%                     Fields:
%                       .save           - Save workspace variables
%                       .viz            - Show offline state plots
%                       .viz_video      - Show offline animation
%                       .run_parallel   - Flag indicating parallel execution
%                       .dynamic_end    - Stop early if agents finish/crash
%
% OUTPUTS:
%   pos_ned_history      - (matrix) [N x 3*num_agents] Position history in NED frame
%                          Format: [n1,e1,d1,n2,e2,d2,...] for each timestep
%
%   vel_ned_history      - (matrix) [N x 3*num_agents] Velocity history in NED frame
%                          Format: [vn1,ve1,vd1,vn2,ve2,vd2,...] for each timestep
%
%   time_history         - (vector) [N x 1] Time vector [s]
%
%   p_swarm              - (struct) Final swarm parameters used in simulation
%
%   map                  - (struct) Map/environment configuration used
%
%   collision_history    - (logical matrix) [N x num_agents] Collision status
%                          True when agent is in collision
%
%   in_sim_history       - (logical matrix) [N x num_agents] Simulation status
%                          False when agent removed from simulation
%
% SIMULATION ENVIRONMENT:
%   The simulation includes:
%   - Arena boundaries (geofence): X=[0,1000]m, Y=[0,500]m, Z=[-75,-10]m
%   - Cylindrical obstacles (buildings) at configurable locations
%   - Spherical obstacles at configurable locations
%   - Initial swarm position: centered at [50,200,-30]m with ±10m randomization
%   - Migration direction: positive X (North)
%   - Default reference speed: 6 m/s
%   - Default collision radius: 0.5 m
%   - Default awareness radius: 30 m
%
% SIMULATION LOOP:
%   For each timestep (dt = 0.01s):
%     1. Check for scheduled failures and apply if needed
%     2. Compute velocity commands from swarm algorithm (every 0.10s)
%     3. Update agent states using quadcopter dynamics
%     4. Check for collisions (agent-agent, agent-obstacle, agent-wall)
%     5. Remove colliding agents from simulation
%     6. Check dynamic end conditions if enabled
%
% FAILURE MODES:
%   Type 1 (Power cutoff): Sets motor max angular velocity to zero
%   Type 2 (Velocity change): Forces agent to new velocity direction/magnitude
%
% COLLISION HANDLING:
%   When collision detected:
%   - Agent is frozen in place (velocity set to zero)
%   - Agent removed from simulation (state_in_sim = false)
%   - Agent excluded from future swarm calculations
%
% OUTPUT FILES (if flags.save = true and results_dir provided):
%   - state_var.mat: Contains all simulation histories and parameters
%   - Offline plots of position, velocity, and acceleration (if flags.viz = true)
%
% PARAMETER FILES LOADED:
%   - param_sim_scenario.m     : Simulation timesteps and durations
%   - param_battery.m          : Battery model parameters
%   - param_physics.m          : Physical constants
%   - param_drone.m            : Quadcopter parameters
%   - param_map_scenario.m     : Environment/obstacle configuration
%   - param_swarm_scenario.m   : Swarm behavioral parameters
%   - param_<algorithm>.m      : Algorithm-specific parameters
%
% NOTES:
%   - Simulation timestep (dt) is fixed at 0.01s for quadcopter dynamics
%   - Command update rate is 0.10s (10 Hz)
%   - Random seed is set to 5 for reproducibility
%   - Visualization and saving are disabled when run_parallel = true
%

%% Specify simulation options and set up workspace 

% Simulation options
% DRONE_TYPE = "point_mass";
DRONE_TYPE = "quadcopter"; % Must set p_sim.dt to 0.01

% Swarm algorithm
if isempty(swarm_config.swarm_algo)
    SWARM_ALGORITHM = "vasarhelyi";
else
    SWARM_ALGORITHM = swarm_config.swarm_algo;
end

% Set number of drones in swarm
p_swarm.nb_agents = swarm_config.num_agents;

% Tolerance for checking failure time (consider time step size of 0.01)
tol_time = 0.001;

% Flags to activate obstacle types
ACTIVE_ENVIRONMENT = true; % this is used to set up obstacle parameters
ACTIVE_OBSTACLES_CYLINDERS = true;
ACTIVE_OBSTACLES_SPHERES = true;
ACTIVE_ARENA_WALLS = true;
ACTIVE_OBSTACLES_BLOCKS = false;

% Additional options
VIDEO = false;
CENTER_VIEW_ON_SWARM = false; % Only used for video

%% Input checks

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

%% Call parameter files
end_time = round(end_time, 2);  % to handle machine error
p_sim.end_time = end_time;
run('param_sim_scenario');
run('param_battery');
run('param_physics');
run('param_drone'); 
run('param_map_scenario');
run('param_swarm_scenario');

% Add extra cylinder if specified
if isfield(fail_config, 'obs_n') && ~isempty(fail_config.obs_n)
    fail_cyl_width = 10;
    map.buildings_width = [map.buildings_width; fail_cyl_width];
    map.buildings_east = [map.buildings_east; fail_config.obs_e]; % Corresponds to Y
    map.buildings_north = [map.buildings_north; fail_config.obs_n]; % Corresponds to X
end

% Check config parameters and set defaults if needed
[p_sim, p_battery, p_physics, p_drone, map, p_swarm] = ...
    check_params(p_sim, p_battery, p_physics, p_drone, map, p_swarm);

%% Init Swarm Object, Wind, Viewer, and other variables

% Init swarm and set positions
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;
for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map, p_swarm);
end
swarm.set_pos(p_swarm.Pos0);

% Init wind
wind = zeros(6,1); % steady wind (1:3), wind gusts (3:6)

% If a video should be saved
if VIDEO    
    % Init video
    video_filename = strcat(erase(mfilename, "example_"), '_', date_string);
    video_filepath = strcat(results_dir, '/', video_filename);
    video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
    
    % Init viewer (this is what will be recorded for video)
    % Check if p_sim.dt_plot and p_sim.dt_video are appropriate
    swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
    swarm_viewer.viewer_type = 'agent';
    states_handle = [];
end

%% Main simulation loop
disp('Type CTRL-C to exit');

% Initialize counter for command updates
command_dt = p_sim.dt_command;  % [sec] command update period
last_command_time = -command_dt;

for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Apply failure at failure time
    swarm = failure_manager(swarm, p_drone, time, fail_config, tol_time);

    % Compute velocity commands from swarming algorithm
    if (time - last_command_time) >= (command_dt - tol_time)
        [~,~] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt_command);
        last_command_time = time;
    end

    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

    % Check for collisions and update agent collision and in_sim states if
    % collisions occur.
    swarm = collision_manager(swarm, p_swarm, map);

    % Update video
    if VIDEO
        swarm_viewer.update(time, swarm, map, p_swarm);
        video.update(time, swarm_viewer.figure_handle);  
    end

    % End sim if all agents cross end_x OR all agents are out of sim
    if flags.dynamic_end
        % Current x positions of all agents
        current_x = [swarm.drones.pos_ned];
        current_x = current_x(1, :);

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

% Close video
if VIDEO
    video.close()
end


%% Analyse states and save

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
if ~flags.run_parallel && flags.save  % cannot save if running in parallel
    workspace_path = strcat(results_dir,'/state_var');
    save(workspace_path,'time_history','pos_ned_history','vel_ned_history', ...
        'accel_history', 'collision_history', 'in_sim_history', ...
        'p_sim', 'p_swarm', 'map');
end

%% Run visualization and postprocessing
if ~flags.run_parallel && flags.viz_video
    % Runs a video but does not save anything.
    SwarmViewerOffline(p_sim.dt_video, ...
        CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map, p_swarm);
end

%% Run their postprocessing (minus saving workspace variables)
if ~flags.run_parallel && flags.viz
    % Analyse swarm state variables

    fontsize = 12;

    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dir);

end

end

%% Local functions

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

            case 1
                % Failure mode: motors max angular velocity set to 0

                % Apply change in max angular velocity
                swarm.drones(vect_fail_now(i)).p_drone.k_omega = 0;

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

    end

end

end

% Manage collisions that occur during simulation
function swarm = collision_manager(swarm, p_swarm, map)
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

