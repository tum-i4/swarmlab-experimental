function out = wrapper_run_scenario_instances(...
    sim_config, swarm_config, sim_fail_config, results_dir, flags)
% WRAPPER_RUN_SCENARIO_INSTANCES Run one or more UAV swarm simulation scenarios
%
% This function serves as a wrapper to execute single or multiple UAV swarm
% simulation runs, either consecutively or in parallel. It manages the setup,
% execution, and output collection for batch simulation experiments.
%
% SYNTAX:
%   out = wrapper_run_scenario_instances()
%   out = wrapper_run_scenario_instances(sim_config, swarm_config, sim_fail_config, results_dir, flags)
%
% INPUTS:
%   sim_config      - (struct) Simulation configuration parameters
%                     Fields (all must be cell arrays of equal length):
%                       .end_time  - Simulation end times [s] 
%                       .start_x   - Starting x-coordinates [m]
%                       .end_x     - Ending x-coordinates [m] 
%
%   swarm_config    - (struct) Swarm configuration parameters
%                     Fields:
%                       .num_agents   - Number of agents in swarm 
%                       .swarm_algo   - Algorithm name: "vasarhelyi"
%                                       or "course_controller"
%
%   sim_fail_config - (struct) Failure scenario configuration
%                     Fields (all must be cell arrays of equal length):
%                       .failure_option - Failure type: 0 (none), 1 (power cutoff),
%                                         2 (velocity change)
%                       .fail_agent_num - Agent number(s) to fail 
%                       .fail_time      - Time(s) of failure [s] 
%                       .obs_n          - Obstacle N-coordinate [m]
%                       .obs_e          - Obstacle E-coordinate [m]
%                     For failure_option = 2, also include:
%                       .fail_v_n       - New velocity north component [m/s]
%                       .fail_v_e       - New velocity east component [m/s]
%                       .fail_v_d       - New velocity down component [m/s]
%
%   results_dir     - (string) Directory path for saving results
%                     If empty, auto-generates timestamped folder 
%
%   flags           - (struct) Simulation control flags
%                     Fields:
%                       .output_all_states - Save full state history
%                       .save              - Save results to file 
%                       .viz               - Show plots 
%                       .viz_video         - Show video 
%                       .dynamic_end       - End early if all agents finish or crash 
%                       .run_parallel      - Run simulations in parallel
%
% OUTPUTS:
%   out             - (struct) Simulation results containing cell arrays, one per run:
%                     .map              - Map/environment configurations
%                     .fail_config      - Failure configurations used
%                     .p_swarm          - Swarm parameters used
%                     .pos_ned          - Position histories [N x 3*num_agents]
%                     .vel_ned          - Velocity histories [N x 3*num_agents]
%                     .time             - Time vectors
%                     .collision        - Collision histories [N x num_agents]
%                     .in_sim           - In-simulation status [N x num_agents]
%                     .times_start      - Times when agents cross start_x
%                     .times_end        - Times when agents cross end_x
%
% NOTES:
%   - Number of runs is determined by the length of cell arrays in sim_config
%   - All cell array fields in sim_config and sim_fail_config must have equal length
%   - Parallel execution requires MATLAB Parallel Computing Toolbox
%   - Results are saved to 'results/results_swarm/<timestamp>/' if
%       results_dir = []
%

%% Inputs specified in this section

%%%%% Set default arguments if inputs are not provided; then the function
%%%%% can be executed by itself.
arguments
    % Simulation configuration
    sim_config struct = struct()
    swarm_config struct = struct()
    sim_fail_config struct = struct()
    results_dir string = []
    flags struct = struct()
end

%%%%% Set defaults if not provided (lists from Python become cell arrays)
if isempty(fieldnames(sim_config))  % sim_config - all inputs as cell
    sim_config.end_time = {150};
    sim_config.start_x = {100};
    sim_config.end_x = {500};
end
if isempty(fieldnames(sim_fail_config))
    % No failures
    sim_fail_config.failure_option = {0};  
    sim_fail_config.fail_agent_num = {NaN}; 
    sim_fail_config.fail_time = {NaN};
    sim_fail_config.obs_n = {[]};
    sim_fail_config.obs_e = {[]};

    % % Failure type 1: power cutoff (comment in if using)
    % sim_fail_config.failure_option = {1};
    % sim_fail_config.fail_agent_num = {1};
    % sim_fail_config.fail_time = {50.0};
    % sim_fail_config.obs_n = {300.0};
    % sim_fail_config.obs_e = {175.0};

    % % Failure type 2: change in agent's direction and speed (comment in if
    % % using)
    % sim_fail_config.failure_option = {2};
    % sim_fail_config.fail_agent_num = {1};
    % sim_fail_config.fail_time = {50.0};
    % sim_fail_config.fail_v_n = {3.0};
    % sim_fail_config.fail_v_e = {3.0};
    % sim_fail_config.fail_v_d = {0.5};
    % sim_fail_config.obs_n = {300.0};
    % sim_fail_config.obs_e = {175.0};
end
if isempty(fieldnames(swarm_config))  % swarm_config - same for all runs
    swarm_config.num_agents = 5;
    swarm_config.swarm_algo = "vasarhelyi";
    % swarm_config.swarm_algo = "course_controller";
end
if isempty(fieldnames(flags))  % flags to control other simulation config
    flags.output_all_states = true;  % set to false to save RAM; set to true if flags.save = true
    flags.save = true;
    flags.viz = true;
    flags.viz_video = false;
    flags.dynamic_end = true; % Stops sim before end_time when all agents cross end_x or have crashed
    flags.run_parallel = false;  % Requires Parallel Processing toolbox
end

%% Do not modify below code

%%%%% Automatically check input sizes
if ~isequal(...
        size(sim_config.end_time), ...
        size(sim_config.start_x), ...
        size(sim_config.end_x), ...
        size(sim_fail_config.failure_option), ...
        size(sim_fail_config.fail_agent_num), ...
        size(sim_fail_config.fail_time) ...
        )
    error('Inputs are not the right size.')
end

% Save how many runs are specified
num_runs = length(sim_config.end_time);

%%%%% Swarm and Sim Configurations

% Additional flag settings that are hard-coded for these experiments
flags.cooperating = true;
flags.ACTIVE_ARENA_WALLS = true;

% Set up fail_config inputs
fail_config_list = cell(num_runs, 1);
for idx = 1:num_runs
    % Create fail_config struct for each run in case different fail configs
    % are used.
    fail_config_list{idx} = structfun(@(x) x{idx}, sim_fail_config, 'UniformOutput', false);
end

%%%%% Set Home Directory and Results Directory

% Change to home directory and add all subdirectories to active path
homeDir = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(homeDir);
addpath(genpath(pwd));

% Remove other project directories in "examples" from active path
subdir_list = strsplit(genpath(fullfile(pwd,'examples')), pathsep);
current_path = fileparts(mfilename('fullpath'));
subdir_list = subdir_list(~strcmp(subdir_list, current_path));
for subdir = subdir_list
    rmpath(subdir{:});
end

% Set up results directory
if isempty(results_dir)
    results_dir = strcat('results/results_swarm');
    date_string = string(datetime('now','Format','yyyy_MM_dd_HH_mm_ss'));
    subfolder = strcat(mfilename, '_', date_string);
    results_dir = strcat(results_dir, '/', subfolder);
end
flags.results_dir = results_dir;
if ~exist(results_dir, 'dir') && (flags.save)
    mkdir(results_dir)
end

%%%%% Run simulation(s) - consecutive or parallel depending on settings

% Initialize outputs
out_map = cell(num_runs, 1);
out_fail_config = cell(num_runs, 1);
out_p_swarm = cell(num_runs, 1);
out_pos_ned = cell(num_runs, 1);
out_vel_ned = cell(num_runs, 1);
out_time = cell(num_runs, 1);
out_collision = cell(num_runs, 1);
out_in_sim = cell(num_runs, 1);
out_times_start = cell(num_runs, 1);
out_times_end = cell(num_runs, 1);

if flags.run_parallel  % Parallel option

    % Print status
    fprintf('%s   Starting %d runs in parallel.\n', ...
        datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.S'), ...
        num_runs);
    fprintf('    NOTE: figures will not be produced when running in parallel.\n')

    parfor i = 1:num_runs
        % Make directory to save results of individual iteration if
        % required
        if flags.save
            iter_results_dir = strcat(results_dir, '/iter', sprintf('%03d', i));
            if ~exist(iter_results_dir, 'dir')
                mkdir(iter_results_dir)
            end
        else
            iter_results_dir = [];
        end

        % Execute sim
        rng(5, 'twister')  % Set seed inside of iteration for consistency
        [pos_ned_history, vel_ned_history, time_history, ...
            p_swarm, map, collision_history, in_sim_history] = run_scenario_instance( ...
            sim_config.end_time{i}, sim_config.end_x{i}, ...
            swarm_config, fail_config_list{i}, ...
            iter_results_dir, flags);

        % Calculate times
        [times_start, times_across_end] = calc_start_end_times(...
            pos_ned_history, time_history, collision_history, ...
            swarm_config.num_agents, ...
            sim_config.start_x{i}, sim_config.end_x{i});

        % Store outputs
        if flags.output_all_states
            out_pos_ned{i} = pos_ned_history;
            out_vel_ned{i} = vel_ned_history;
        end
        out_time{i} = time_history';
        out_map{i} = map;
        out_collision{i} = collision_history;
        out_in_sim{i} = in_sim_history;
        out_times_start{i} = times_start;
        out_times_end{i} = times_across_end;
        out_fail_config{i} = fail_config_list{i};
        out_p_swarm{i} = p_swarm;
        
    end


else  % Consecutive option

    % Print status
    fprintf('%s   Starting consecutive runs.\n', ...
        datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.S'));

    for i = 1:num_runs
        % Status update
        fprintf('           %s   Starting run %d of %d.\n', ...
            datetime('now', 'Format', 'HH:mm:ss.S'), ...
            i, num_runs);

        % Make directory to save results of individual iteration if
        % required
        if flags.save
            iter_results_dir = strcat(results_dir, '/iter', sprintf('%03d', i));
            if ~exist(iter_results_dir, 'dir')
                mkdir(iter_results_dir)
            end
        else
            iter_results_dir = [];
        end

        % Execute sim
        rng(5, 'twister')  % Set seed inside of iteration for consistency
        [pos_ned_history, vel_ned_history, time_history, ...
            p_swarm, map, collision_history, in_sim_history] = run_scenario_instance( ...
            sim_config.end_time{i}, sim_config.end_x{i}, ...
            swarm_config, fail_config_list{i}, ...
            iter_results_dir, flags);

        % Calculate times
        [times_start, times_across_end] = calc_start_end_times(...
            pos_ned_history, time_history, collision_history, ...
            swarm_config.num_agents, ...
            sim_config.start_x{i}, sim_config.end_x{i});

        % Store outputs
        if flags.output_all_states
            out_pos_ned{i} = pos_ned_history;
            out_vel_ned{i} = vel_ned_history;
        end
        out_time{i} = time_history';
        out_map{i} = map;
        out_collision{i} = collision_history;
        out_in_sim{i} = in_sim_history;
        out_times_start{i} = times_start;
        out_times_end{i} = times_across_end;
        out_fail_config{i} = fail_config_list{i};
        out_p_swarm{i} = p_swarm;

    end

end


%%%%% Organize outputs and save if required:
out.map = out_map;
out.fail_config = out_fail_config;
out.p_swarm = out_p_swarm;
out.pos_ned = out_pos_ned;
out.vel_ned = out_vel_ned;
out.time = out_time;
out.collision = out_collision;
out.in_sim = out_in_sim;
out.times_start = out_times_start;
out.times_end = out_times_end;

if flags.save
    save(strcat(results_dir, '/output'), 'out');
end

fprintf('%s   Runs completed.\n', ...
    datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.S'));

end


%% Local functions

% Calculate starting time and end_x crossing time
function [times_start, times_across_end] = calc_start_end_times(...
    pos_ned_history, time_history, collision_history, ...
    num_agents, start_x, end_x)

% Extract x positions for all agents (pos_ned_history contains [x1,y1,z1,x2,y2,z2,...])
pos_x = pos_ned_history(:, 1:3:end);  % Extract every 3rd column starting from 1

% Initialize output vectors
times_start = inf(1, num_agents);
times_across_end = inf(1, num_agents);

% For each agent, find when they cross start_x and end_x. Skip end time
% if they had a collision before they crossed end_x.
for agent_idx = 1:num_agents
    agent_x_history = pos_x(:, agent_idx);
    
    % Find first time agent crosses start_x
    start_crossing_indices = find(agent_x_history > start_x, 1);
    if ~isempty(start_crossing_indices)
        times_start(agent_idx) = time_history(start_crossing_indices);
    end
    
    % Find first time agent crosses end_x
    end_crossing_indices = find(agent_x_history > end_x, 1);

    % Save time that agent crosses end_x if no collision occurred prior
    if ~isempty(end_crossing_indices) && ...
            ~any(collision_history(1:end_crossing_indices, agent_idx))
        times_across_end(agent_idx) = time_history(end_crossing_indices);
    end
end

end
