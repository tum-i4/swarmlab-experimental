%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generic parameters for the simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_sim.dt = 0.01; % simulation dt [s] (must be 0.01 to use quadcopter)
p_sim.dt_plot = 0.5; % period of plots update [s]
p_sim.dt_video = 0.1; % period of video update [s]
p_sim.start_time = 0; % [s]
if ~isfield(p_sim, 'end_time')
    p_sim.end_time = 120; % [s]
end
p_sim.dt_command = 0.10;

