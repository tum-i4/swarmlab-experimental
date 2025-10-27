function [vel_command, collisions] = compute_course_controller(self, p_swarm, r_agent, dt)
    % This is where swarm controller logic should be added.
    %
    % Inputs:
    %   p_swarm: swarm parameters
    %   r_agent: safety radius of agents
    %   dt: time step
    %
    % Outputs:
    %   vel_command: commanded velocities for every agent
    %   collisions: [nb_agent_collisions nb_obs_collisions min_dist_obs]
    %

    % Initialize variables
    pos = self.get_pos_ned();           % [3xn] matrix of positions
    vel = self.get_vel_ned();           % [3xn] matrix of velocities
    nb_agents = self.nb_agents;         % n agents
    
    vel_command = zeros(3, nb_agents);  % Total commanded velocity
    collisions = nan(1, 3); % Ignore the collisions output

    % Dummy data
    vel_command(1,:) = 6;

end