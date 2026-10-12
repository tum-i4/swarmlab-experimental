% Goal: use ACTIVE GOAL in combination with manually selecting and dropping
% drones out of the swarm calculations at specific times; drones that "drop
% out" need to be configured to run

% Run this in the SwarmLab home directory.

%% Specify simulation options and set up workspace 

close all
clearvars -except flag_run_viz_post flag_run_video_post set_drone_type

% Option to run visualization and postprocessing (used by test scripts)
if ~exist('flag_run_viz_post','var')
    flag_run_viz_post = true;
end
if ~exist('flag_run_video_post','var')
    flag_run_video_post = true;
end
% flag_run_viz_post = true; % Use this if need to force flag

% Simulation options
if ~exist('set_drone_type','var')
    % DRONE_TYPE = "point_mass"; % swarming mode supports only quadcopter and point_mass
    DRONE_TYPE = "quadcopter";
else
    DRONE_TYPE = set_drone_type; % Option used by test scripts
end

% Set number of drones in swarm
p_swarm.nb_agents = 5; % (will default to 5 in param_swarm if not set here)
% p_swarm.nb_agents = 25; % Interesting to see: collisions will occur with cylinder obstacle modification

% File name for custom map parameter file
% str_param_map = 'param_map'; % use default cylinder obstacle locations
% str_param_map = 'param_map_adjust_cyl_obstacles';
% str_param_map = 'param_map_add_spheres_obstacles'; % updated to define spheres
str_param_map = 'param_map_add_walls'; % updated to define walls

% File name for custom swarm parameter file
% str_param_swarm = 'param_swarm'; % use default swarm parameters
% str_param_swarm = 'param_swarm_add_spheres_obstacles';
% str_param_swarm = 'param_swarm_add_walls';
str_param_swarm = 'param_swarm_active_goal';

% Flags to activate obstacle types
ACTIVE_OBSTACLES_CYLINDERS = true;
ACTIVE_OBSTACLES_SPHERES = true;
ACTIVE_ARENA_WALLS = true;

ACTIVE_ENVIRONMENT = true; % this is used to set up obstacle parameters
% DEBUG = true; 
VIDEO = false;
CENTER_VIEW_ON_SWARM = false; % Only used for video
SWARM_ALGORITHM = "vasarhelyi"; % either vasarhelyi or olfati_saber



%% Directory setup

% Change to home directory and add all subdirectories to active path
homeDir = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(homeDir);
addpath(genpath(pwd));

% For results to be saved to
results_dirname = strcat('results/results_swarm');
date_string = string(datetime('now','Format','yyyy_MM_dd_HH_mm_ss'));
subfolder = strcat(erase(mfilename,"example_"), '_', date_string);
results_dirname = strcat(results_dirname, '/', subfolder);
if ~exist(results_dirname, 'dir')
    mkdir(results_dirname)
end

%% Start timer
tic

%% Call parameter files
p_sim.end_time = 250;  % Clustering around goal is more obvious with extended sim time
run('param_sim');
run('param_battery');
run('param_physics');
run('param_drone'); 
run(str_param_map);
run(str_param_swarm);

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

% Init variables for history
x0 = [p_swarm.Pos0; zeros(3,p_swarm.nb_agents)];
x_history(1,:) = x0(:);

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

disp('Type CTRL-C to exit');
for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Test out new feature - outsider with active goal
    if time == 90
        swarm.drones(2).swarm_in_calcs = false;
        swarm.drones(2).swarm_control = 'goal';
        swarm.drones(2).v_ref = 4;
        swarm.drones(2).x_goal = [250; 250; -10];
    end

    % Test out new feature - outsider with velocity goal
    if time == 90
        swarm.drones(4).swarm_in_calcs = false;
        swarm.drones(4).swarm_control = 'velocity';
        swarm.drones(4).v_ref = 2;
        swarm.drones(4).u_ref = [0; 0; 1];
    end

    % Test out new feature - outsider with delay in exclusion from swarm
    % calculations
    if time == 60
        swarm.drones(1).v_ref = 1;
    elseif time == 70
        swarm.drones(1).swarm_in_calcs = false;
    end

    % Test out new feature - drone goes into failsafe, others remove it
    % from swarm calcs after 10 seconds
    if time == 10
        swarm.drones(3).state_failsafe = true;
        swarm.drones(3).swarm_control = 'velocity';
        swarm.drones(3).u_ref = [1; 0; 0];
        swarm.drones(3).v_ref = 1;
    elseif time == 30
        swarm.drones(3).swarm_in_calcs = false;
    end

    % Test out new feature - remove drone from sim
    if time == 50
        swarm.drones(5).state_in_sim = false;
        swarm.drones(5).u_ref = [0; 0; 0];
        swarm.drones(5).v_ref = 0;
    end


    % Compute velocity commands from swarming algorithm
    [vel_c,collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);

    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

    % Update video
    if VIDEO
        swarm_viewer.update(time, swarm, map, p_swarm);
        video.update(time, swarm_viewer.figure_handle);  
    end

end

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

%% Stop timer
toc

% keyboard % breakpoint; 'dbcont' to continue and run postprocessing, otherwise 'dbquit'

%% Run visualization and postprocessing
if flag_run_viz_post

    fontsize = 12;

    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dirname);

end
if flag_run_video_post

    % This is a cool visualization tool of theirs, but I don't think it saves anything

    SwarmViewerOffline(p_sim.dt_video, ...
        CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map, p_swarm);

end
