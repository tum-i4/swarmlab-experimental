function out = wrapper_run_generation_exp_failure_modes(...
    sim_config, swarm_config, sim_fail_config, results_dir, flags)
% Wrapper specifically designed for failure modes experiments.

%%%%% Set default arguments if some/all inputs are not provided
arguments
    % Simulation configuration
    sim_config struct = struct()
    swarm_config struct = struct()
    sim_fail_config struct = struct()
    results_dir string = []
    flags struct = struct()
end


%%%%% Set defaults if not provided (cell arrays support lists from Python)
if isempty(fieldnames(sim_config))  % sim_config - all inputs as cell
    % test
    sim_config.end_time = {300, 300};
    sim_config.start_x = {0, 0};
    sim_config.end_x = {100, 100};
    % environment config type:
    %   0 = use rerun/validation data
    %   1 = manual selection from entire list of maps and starts
    %   2 = no obstacles, random starts
    sim_config.env_config_type = {0, 1};  % 0=use rerun/validation data, 1=manual selection from entire list of maps and starts
    sim_config.env_maps = {1, 2};
    sim_config.env_starts = {1, 2};
end
if isempty(fieldnames(sim_fail_config))  % fail config - all inputs as cell
    % test
    sim_fail_config.failure_option = {0, 1};
    sim_fail_config.fail_agent_num = {NaN, [1, 2]};
    sim_fail_config.fail_time = {NaN, [25, 35]};
    sim_fail_config.fail_power = {NaN, [0.5, 0.8]};
    sim_fail_config.fail_in_swarm = {true, true};
end
if isempty(fieldnames(swarm_config))  % swarm_config - same for all runs
    swarm_config.num_agents = 12;
    swarm_config.d_ref = 10;
    swarm_config.v_ref = 6;
    swarm_config.r = inf;  % Agent awareness radius for obstacles and other UAVs
    % swarm_config.r = 20; 
end
if isempty(fieldnames(flags))  % flags - same for all runs
    flags.output_all_states = true;  % set to false to save RAM
    flags.save = true;
    flags.viz = true;
    flags.viz_video = true;
    % flags.viz = false;
    % flags.viz_video = false;
    flags.dynamic_end = true;
    flags.run_parallel = false;
    % flags.run_parallel = true;
end

% Before proceeding, check that inputs sizes match
if ~isequal(...
        size(sim_config.end_time), ...
        size(sim_config.start_x), ...
        size(sim_config.end_x), ...
        size(sim_config.env_config_type), ...
        size(sim_config.env_maps), ...
        size(sim_config.env_starts), ...
        size(sim_fail_config.failure_option), ...
        size(sim_fail_config.fail_agent_num), ...
        size(sim_fail_config.fail_time), ...
        size(sim_fail_config.fail_in_swarm) ...
        )
    error('Input sizes do not match.')
else
    % Save how many runs are specified
    num_runs = length(sim_config.end_time);

    % Need to check input sizes for failure configs
    if isfield(sim_fail_config, 'fail_power') && ...
            isequal(num_runs, length(sim_fail_config.fail_power))
        % all good, do nothing
    elseif (isfield(sim_fail_config, 'fail_v_n') || ...
            isfield(sim_fail_config, 'fail_v_e') || ...
            isfield(sim_fail_config, 'fail_v_d')) ...
            && ...
            isequal(num_runs, ...
            length(sim_fail_config.fail_v_n), ...
            length(sim_fail_config.fail_v_e), ...
            length(sim_fail_config.fail_v_d))
        % all good, do nothing
    else
        error('Fail configuration inputs are not the right size or are not recognized.')
    end
end


%%%%% Set correct paths to required data

% Paths to files with tuning and map data
path_maps_1sph_1cyl = fullfile("projects\robustness-failures-obstacles\data\20250822_144327_predefined_maps_obst_1sph_1cyl_100trials_50cross.mat");
path_starts_3to150 = fullfile("projects\robustness-failures-obstacles\data\20250820_181508_predefined_starts_3_to_150_50trials_50cross.mat");
path_maps_obst_0 = fullfile("projects\robustness-failures-obstacles\data\20250820_181414_predefined_maps_obst_0_1trials_50cross.mat");

% Updated model
path_tuning_rInf = fullfile("projects\robustness-failures-obstacles\data\rInf_script_run_tuning_2025_10_08_13_28_09\tuning_final_results.mat");
path_validation_rInf = fullfile("projects\robustness-failures-obstacles\data\rInf_script_run_validation_2025_10_09_14_36_26\validation_final_results.mat");

path_tuning_r20 = fullfile("projects\robustness-failures-obstacles\data\r20_script_run_tuning_2025_08_29_20_36_59\tuning_final_results.mat");
path_validation_r20 = fullfile("projects\robustness-failures-obstacles\data\r20_script_run_validation_2025_09_01_15_20_25\validation_final_results.mat");

tol_r = 1e-4;
if swarm_config.r == inf
    path_tuning = path_tuning_rInf;
    path_validation = path_validation_rInf;
elseif abs(swarm_config.r - 20) < tol_r
    path_tuning = path_tuning_r20;
    path_validation = path_validation_r20;
else
    error('The specified agent awareness range does not have an associated tuning result.')
end


%%%%% Swarm and Sim Configurations

% Additional flag settings that are hard-coded for these experiments
flags.cooperating = true;
flags.ACTIVE_ARENA_WALLS = true;

% Set up fail_config inputs
fail_config_list = cell(num_runs, 1);
for idx = 1:num_runs
    % Create fail_config struct for each run
    fail_config_list{idx} = structfun(@(x) x{idx}, sim_fail_config, 'UniformOutput', false);
end

% Load Vasarhelyi config from tuning results
load(path_tuning, 'opt_outputs')
p_vasar.d_ref =     swarm_config.d_ref;
p_vasar.p_rep =     opt_outputs.x_pop(1);
p_vasar.r0_fric =   opt_outputs.x_pop(2);
p_vasar.C_fric =    opt_outputs.x_pop(3);
p_vasar.v_fric =    opt_outputs.x_pop(4);
p_vasar.p_fric =    opt_outputs.x_pop(5);
p_vasar.a_fric =    opt_outputs.x_pop(6);
p_vasar.r0_shill =  opt_outputs.x_pop(7);
p_vasar.v_shill =   opt_outputs.x_pop(8);
p_vasar.p_shill =   opt_outputs.x_pop(9);
p_vasar.a_shill =   opt_outputs.x_pop(10);


%%%%% Load relevant maps and starting positions

[maps_list, starts_list] = load_maps_start(swarm_config.num_agents, ...
    path_validation, ...  % Type 0: use for rerun/validation results
    path_maps_1sph_1cyl, path_starts_3to150, ...  % Type 1: manual selection
    path_maps_obst_0, ...  % Type 2: no obstacles
    [sim_config.env_config_type{:}], ...
    [sim_config.env_maps{:}], ...
    [sim_config.env_starts{:}]);


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

% Remove other project directories in "projects" from active path
subdir_list = strsplit(genpath(fullfile(pwd,'projects')), pathsep);
current_path = fileparts(mfilename('fullpath'));
parent_path = fileparts(current_path);
subdir_list = subdir_list(~strcmp(subdir_list, current_path));
for subdir = subdir_list
    if ~isequal(subdir{:}, parent_path)
        rmpath(subdir{:});
    end
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
            p_swarm, map, collision_history, in_sim_history] = run_sim_once(...
            sim_config.end_time{i}, sim_config.end_x{i}, ...
            swarm_config, maps_list{i}, starts_list(i), p_vasar, ...
            fail_config_list{i}, iter_results_dir, flags);

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
            p_swarm, map, collision_history, in_sim_history] = run_sim_once(...
            sim_config.end_time{i}, sim_config.end_x{i}, ...
            swarm_config, maps_list{i}, starts_list(i), p_vasar, ...
            fail_config_list{i}, iter_results_dir, flags);

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

% Load maps from rerun and validation runs
function [maps, starts] = load_maps_start(num_agents, ...
    path_validate, ...  % type = 0
    path_maps, path_starts, ...  % type = 1
    path_maps_obs0, ...   % type = 2; also uses path_starts
    type, idx_maps, idx_starts)  % these should be vectors

% First check that input sizes match
if ~isequal(size(type), size(idx_maps), size(idx_starts))
    error('Input sizes do not match.')
else
    num_env = length(type);
end

% Initialize output
maps = cell(num_env, 1);
starts(num_env) = struct('agent_positions', [], ...  % store starting locations
    'start_buffer', [], 'Lx', [], 'Ly', [], 'Lz', []);  

% Load data if types call for it
unique_types = unique(type);

% Type 0: Load rerun and validation data if needed
if any(unique_types == 0)
    % Load reruns and validation runs to get maps
    load(path_validate, 'fitness_rerun', 'fitness_validate')

    % Get list of maps and starts from the loaded file (reruns and validation
    % runs)
    maps_list = [fitness_rerun.ex_inputs.maps_sample; ...
        fitness_validate.ex_inputs.maps_sample];
    starts_list = [fitness_rerun.ex_inputs.starts_sample, ...
        fitness_validate.ex_inputs.starts_sample];
    f_list = [fitness_rerun.f; fitness_validate.f];

    % Exclude the ones with fitness of 0
    is_valid = f_list > 0;
    maps_list = maps_list(is_valid);
    starts_list = starts_list(is_valid);
end

% Type 1: Load all maps and starts data if needed
if any(unique_types == 1)
    % Load all maps
    load(path_maps, "save_maps")

    % Load all starts
    load(path_starts, "swarm_sizes", "start_buffer", "Lx", "Ly", "Lz", ...
        "agent_positions")
end

% Type 2: Load map with no obstacles and starts data
if any(unique_types == 2)
    % Load map with no obstacles
    load(path_maps_obs0, "save_maps")

    % Load all starts (if not already loaded)
    if ~exist('agent_positions', 'var')
        load(path_starts, "swarm_sizes", "start_buffer", "Lx", "Ly", "Lz", ...
        "agent_positions")
    end
end

% Iterate through each requested config
for i = 1:num_env

    switch type(i)
        case 0  % If type = 0, load rerun and validation results
            % Check size matches number of agents
            temp_num_agents = size(starts_list(idx_starts(i)).agent_positions, 1);
            if ~isequal(num_agents, temp_num_agents)
                error('Specified starting positions do not match swarm size.')
            end

            % Select starts and maps
            starts(i) = starts_list(idx_starts(i));
            maps{i} = maps_list{idx_maps(i)};

        case 1  % Manual selection of maps and starts
            % Select starts based on user-provided idx and num_agents
            temp_idx_num_agents = find(swarm_sizes == num_agents);
            if isempty(temp_idx_num_agents)
                error('Specified swarm size not found in provided starts data.')
            end
            starts(i).start_buffer = start_buffer;
            starts(i).Lx = Lx;
            starts(i).Ly = Ly;
            starts(i).Lz = Lz;
            starts(i).agent_positions = agent_positions{temp_idx_num_agents, ...
                idx_starts(i)};

            % Select map
            maps{i} = save_maps.maps{idx_maps(i)};

        case 2  % Map with no obstacles, user-selected starts
            % Select starts based on user-provided idx and num_agents
            temp_idx_num_agents = find(swarm_sizes == num_agents);
            if isempty(temp_idx_num_agents)
                error('Specified swarm size not found in provided starts data.')
            end
            starts(i).start_buffer = start_buffer;
            starts(i).Lx = Lx;
            starts(i).Ly = Ly;
            starts(i).Lz = Lz;
            starts(i).agent_positions = agent_positions{temp_idx_num_agents, ...
                idx_starts(i)};

            % Select map
            maps{i} = save_maps.maps{1};  % Only one map with no obstacles

        otherwise
            error('Please specify a valid environment type designation.')
    end

end

end


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

