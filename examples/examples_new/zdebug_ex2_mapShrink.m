% Goal: try to move cylinder obstacles to locations that I specify.

% Run this in the SwarmLab home directory.

%% Set up simulation options and workspace (figures will be closed, variables cleared)

close all
clearvars -except flag_run_viz_post set_drone_type

% Option to run visualization and postprocessing (used by test scripts)
if ~exist('flag_run_viz_post','var')
    flag_run_viz_post = true;
end
% flag_run_viz_post = true; % Use this if need to force flag

% Simulation options
if ~exist('set_drone_type','var')
    DRONE_TYPE = "point_mass"; % swarming mode supports only quadcopter and point_mass
    % DRONE_TYPE = "quadcopter";
else
    DRONE_TYPE = set_drone_type; % Option used by test scripts
end

% Set number of drones in swarm
p_swarm.nb_agents = 5; % (will default to 5 in param_swarm if not set here)
% p_swarm.nb_agents = 25; % Interesting to see: collisions will occur

% File name for custom map parameter file
str_param_map = 'param_map_adjust_cyl_obstacles_2_2';
% str_param_map = 'param_map'; % use default cylinder obstacle locations

ACTIVE_ENVIRONMENT = true; % this is used to set up obstacle parameters
% DEBUG = true; % not creating a video, so try commenting out
% VIDEO = true; % not creating a video, so try commenting out
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
run('param_swarm_ex2_2');

%% Init Swarm Object, Wind, Viewer, and other variables

% Init swarm and set positions
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;

for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map);
end
swarm.set_pos(p_swarm.Pos0);

% Init wind
wind = zeros(6,1); % steady wind (1:3), wind gusts (3:6)

% Init variables for history
x0 = [p_swarm.Pos0; zeros(3,p_swarm.nb_agents)];
x_history(1,:) = x0(:);

% % Init video
% if VIDEO    
%     video_filename = strcat(erase(mfilename, "example_"), '_', date_string);
%     video_filepath = strcat(results_dirname, '/', video_filename);
%     video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
% end
% 
% % Init viewer
% swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
% swarm_viewer.viewer_type = SWARM_VIEWER_TYPE;
% states_handle = [];

%% Main simulation loop

disp('Type CTRL-C to exit');
for time = p_sim.start_time:p_sim.dt:p_sim.end_time

%     % Check if program terminated from GUI
%     if exist('app', 'var')
%         switch app.StartsimulationSwitch.Value
%             case 'Off'
%                 close all;
%                 return;
%         end
%     end

%     % Get change from GUI
%     if exist('app', 'var')
%         % Wind parameters
%         wind_active = app.wind;
%         wind_gust_active = app.wind_gust;
%         wind_level = app.wind_level;
%         wind_gust_level = app.wind_gust_level;
%         
%         % Debug plot
%         debug_plot = app.debug_plot;
%         
%         % Orientation of swarm migration
%         orientation = app.orientation;
%         p_swarm.u_ref = [-cosd(orientation), -sind(orientation), 0]';
%     end

    % Compute velocity commands from swarming algorithm
    [vel_c,collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);

    % Update swarm states and plot the drones
    swarm.update_state(wind, time);

%     % Plot state variables for debugging
%     if DEBUG
%     swarm.plot_state(time, p_sim.end_time, ...
%         1, p_sim.dt_plot, collisions, p_swarm.r_coll/2);
%     end

%     % Update video
%     if VIDEO
%         swarm_viewer.update(time, swarm, map);
%         video.update(time, swarm_viewer.figure_handle);  
%     end

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

%% Stop timer
toc

% keyboard % breakpoint; 'dbcont' to continue and run postprocessing, otherwise 'dbquit'

%% Run visualization and postprocessing
if flag_run_viz_post

    % This is a cool visualization tool of theirs, but I don't think it saves anything
    SwarmViewerOffline(p_sim.dt_video, ...
        CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map, p_swarm);

    % Run their postprocessing (minus saving workspace variables)

    % Analyse swarm state variables

    fontsize = 12;

    time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
    pos_ned_history = swarm.get_pos_ned_history();
    pos_ned_history = pos_ned_history(2:end,:);
    vel_ned_history = swarm.get_vel_xyz_history();
    accel_history = [zeros(1, p_swarm.nb_agents*3); ...
        diff(vel_ned_history,1)/p_sim.dt];

    %     % Save workspace
    %     wokspace_path = strcat(results_dirname,'/state_var');
    %     save(wokspace_path,'time_history','pos_ned_history','vel_ned_history', ...
    %         'accel_history');

    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dirname);


    %%Analyse performance

    % Compute swarm performance
    [safety, order, union, alg_conn, safety_obs, min_d_obs] = ...
        compute_swarm_performance(pos_ned_history, vel_ned_history, ...
        p_swarm, results_dirname);

    % Plot performance
    [perf_handle] = plot_swarm_performance(time_history', safety, order, ...
        union, alg_conn, safety_obs, min_d_obs, p_swarm, fontsize, results_dirname);

end
