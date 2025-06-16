% Example of a point-to-point flight segment

%% Specify simulation options and set up workspace 

close all
clearvars

% Option to run visualization and postprocessing (used by test scripts)
flag_run_viz_post = true;
% flag_run_viz_post = false;

% Simulation options
DRONE_TYPE = "point_mass";
% DRONE_TYPE = "quadcopter"; % Must set p_sim.dt to 0.01

% Set number of drones in swarm
p_swarm.nb_agents = 5;

% Flags to activate obstacle types
ACTIVE_ENVIRONMENT = true; % this is used to set up obstacle parameters
ACTIVE_OBSTACLES_CYLINDERS = true;
ACTIVE_OBSTACLES_SPHERES = true;
ACTIVE_ARENA_WALLS = true;
ACTIVE_OBSTACLES_BLOCKS = true;

% Additional options
DEBUG = false; 
VIDEO = false;
CENTER_VIEW_ON_SWARM = false; % Only used for video
SWARM_ALGORITHM = "vasarhelyi";

%% Directory setup

% Change to home directory and add all subdirectories to active path
homeDir = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(homeDir);
addpath(genpath(pwd));

% For results to be saved to correct location
results_dirname = strcat('results/results_swarm');
date_string = string(datetime('now','Format','yyyy_MM_dd_HH_mm_ss'));
subfolder = strcat(mfilename, '_', date_string);
results_dirname = strcat(results_dirname, '/', subfolder);
if ~exist(results_dirname, 'dir')
    mkdir(results_dirname)
end

%% Call parameter files
p_sim.end_time = 300;  % Clustering around goal is more obvious with extended sim time
run('param_sim_scenario');
run('param_battery');
run('param_physics');
run('param_drone'); 
run('param_map_scenario');
run('param_swarm_scenario');

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
    video_filepath = strcat(results_dirname, '/', video_filename);
    video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
    
    % Init viewer (this is what will be recorded for video)
    % Check if p_sim.dt_plot and p_sim.dt_video are appropriate
    swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
    swarm_viewer.viewer_type = 'agent';
    states_handle = [];
end

%% Main simulation loop
tic % start timer
disp('Type CTRL-C to exit');

for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Compute velocity commands from swarming algorithm
    [vel_c,collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);

    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

    % Plot state variables for debugging
    if DEBUG
        swarm.plot_state(time, p_sim.end_time, ...
            1, p_sim.dt_plot, collisions, p_swarm.r_coll/2);
    end

    % Update video
    if VIDEO
        swarm_viewer.update(time, swarm, map, p_swarm);
        video.update(time, swarm_viewer.figure_handle);  
    end

end
toc %stop timer, display

%% Analyse states and save

time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
pos_ned_history = swarm.get_pos_ned_history();
pos_ned_history = pos_ned_history(2:end,:);
vel_ned_history = swarm.get_vel_xyz_history();
accel_history = [zeros(1, p_swarm.nb_agents*3); ...
    diff(vel_ned_history,1)/p_sim.dt];

% Save workspace
wokspace_path = strcat(results_dirname,'/state_var');
save(wokspace_path,'time_history','pos_ned_history','vel_ned_history', ...
    'accel_history');

% Close video
if VIDEO
    video.close()
end

%% Run visualization and postprocessing
if flag_run_viz_post

    % This is a cool visualization tool of theirs, but I don't think it saves anything
    SwarmViewerOffline(p_sim.dt_video, ...
        CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map, p_swarm);

    %% Run their postprocessing (minus saving workspace variables)

    % Analyse swarm state variables

    fontsize = 12;

    % time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
    % pos_ned_history = swarm.get_pos_ned_history();
    % pos_ned_history = pos_ned_history(2:end,:);
    % vel_ned_history = swarm.get_vel_xyz_history();
    % accel_history = [zeros(1, p_swarm.nb_agents*3); ...
    %     diff(vel_ned_history,1)/p_sim.dt];

    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dirname);

end