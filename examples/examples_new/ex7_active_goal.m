% Goal: try to use the ACTIVE GOAL parameter to specify goal location 
%       instead of migration. Note that simulation time is longer than the
%       other examples.

% Run this in the SwarmLab home directory.

% Environment configuration in this example should be the same as EX4.
% Using the ACTIVE GOAL flight mode results in slightly different behavior
% compared to ACTIVE MIGRATION. Additionally, this example uses the
% "quadcopter" model as that is more "realistic" and results in less noise
% in calculations. Note that the tradeoff is computation time.

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
CENTER_VIEW_ON_SWARM = false;
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
% if DRONE_TYPE == "fixed_wing" || DRONE_TYPE == "quadcopter"
%     run('param_drone'); 
% elseif DRONE_TYPE == "point_mass"
%     run('param_drone'); 
% end
    run('param_drone'); % seems like I can just replace the above with this...
run(str_param_map); % creates map: struct for map params
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

%% Main simulation loop

disp('Type CTRL-C to exit');
for time = p_sim.start_time:p_sim.dt:p_sim.end_time

    % Compute velocity commands from swarming algorithm
    [vel_c,collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);

    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

end

%% Analyse states and save

time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
pos_ned_history = swarm.get_pos_ned_history();
pos_ned_history = pos_ned_history(2:end,:);
vel_xyz_history = swarm.get_vel_xyz_history();
vel_ned_history = swarm.get_vel_ned_history();
accel_history = [zeros(1, p_swarm.nb_agents*3); ...
    diff(vel_ned_history,1)/p_sim.dt];

% Save workspace
wokspace_path = strcat(results_dirname,'/state_var');
save(wokspace_path,'time_history','pos_ned_history','vel_ned_history', ...
    'vel_xyz_history', 'accel_history');

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
