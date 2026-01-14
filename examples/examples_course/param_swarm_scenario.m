% In your main, run this script after the swarm initialization

% Variables to be set
p_swarm.is_active_migration = true;
p_swarm.is_active_goal = false;
p_swarm.is_active_arena = ACTIVE_ARENA_WALLS;
p_swarm.is_active_spheres = ACTIVE_OBSTACLES_SPHERES;
p_swarm.is_active_cyl = ACTIVE_OBSTACLES_CYLINDERS;
p_swarm.is_active_blocks = ACTIVE_OBSTACLES_BLOCKS;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number of agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isfield(p_swarm, 'nb_agents')
    p_swarm.nb_agents = 5;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max radius of influence - Metric distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r = 30;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Max number of neighbors - Topological distance
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isfield(p_swarm, 'max_neig')
    p_swarm.max_neig = 10;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radius of collision -
% it is the radius of the sphere that approximates
% the drone. A collision is counted when two 
% spheres intersect.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p_swarm.r_coll = 0.5;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arena parameters - Cubic arena
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') && ACTIVE_ENVIRONMENT && p_swarm.is_active_arena)
    p_swarm.x_arena = [map.arena_north; map.arena_east; map.arena_down];
    p_swarm.center_arena = sum(p_swarm.x_arena, 2) / 2;
end

% Parameter that defines the influence radius of the arena repulsion force
p_swarm.d_arena = 1.5;

% Constant of proportionality of the arena repulsion force
p_swarm.c_arena = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Spheric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') && ACTIVE_ENVIRONMENT && p_swarm.is_active_spheres)
    p_swarm.spheres = [map.spheres_north; map.spheres_east; ...
        map.spheres_down; map.spheres_r];
    p_swarm.n_spheres = map.n_spheres;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cylindric obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (exist('map','var') && ACTIVE_ENVIRONMENT && p_swarm.is_active_cyl)
    p_swarm.cylinders = [
        map.buildings_north'; % x_obstacle
        map.buildings_east'; % y_obstacle
        (map.buildings_width./2)']; % r_obstacle
    p_swarm.n_cyl = length(p_swarm.cylinders(1, :));
else
    p_swarm.cylinders = 0;
    p_swarm.n_cyl = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Block obstacles parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add extra cylinder if specified
if isfield(fail_config, 'obs_n') && ~isempty(fail_config.obs_n)
    fail_cyl_width = 10;

    % --- add to p_swarm.cylinders: 3×N -> [n_row; e_row; width_row] ---
    % make sure these are row vectors
    new_cyl_n = fail_config.obs_n(:).';                 % 1×K
    new_cyl_e = fail_config.obs_e(:).';                 % 1×K
    new_cyl_w = fail_cyl_width * ones(1, numel(new_cyl_n));  % 1×K

    new_cyl = [new_cyl_n;
               new_cyl_e;
               new_cyl_w];                              % 3×K

    if isfield(p_swarm, 'cylinders') && ~isempty(p_swarm.cylinders)
        % existing 3×N, append new 3×K → 3×(N+K)
        p_swarm.cylinders = [p_swarm.cylinders, new_cyl];
    else
        % initialize
        p_swarm.cylinders = new_cyl;
    end

    % update number of cylinders (number of columns)
    p_swarm.n_cyl = size(p_swarm.cylinders, 2);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference values
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inter-agent distance
if ~isfield(p_swarm, 'd_ref')
    p_swarm.d_ref = 15;
end

% Velocity direction
p_swarm.u_ref = [1 0 0]';

% Speed
if ~isfield(p_swarm, 'v_ref')
    p_swarm.v_ref = 6;
end

% Goal location
p_swarm.x_goal = [400, 400, -20]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity and acceleration bounds for the agents
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if DRONE_TYPE == "fixed_wing" || DRONE_TYPE == "quadcopter"
    p_swarm.max_a = []; % leave empty if you use a real drone model
elseif DRONE_TYPE == "point_mass"
    p_swarm.max_a = 10;
else
	error('DRONE_TYPE is incorrectly specified.')
end
p_swarm.max_v = 7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial position and velocity for the swarm
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUG was HERE
% Initial positions are contained in a cubic area
p_swarm.P0 = [50,200,-30]'; % [m] position of cube center
p_swarm.P = 20; % [m] cube edge size

% Velocities are inizialized in a cubic subspace
p_swarm.V0 = [0,0,0]'; % [m/s]
p_swarm.V = 0; % [m/s]

% Seed to avoid random effects
p_swarm.seed = 5;
rng(p_swarm.seed);

% ---- spawn with minimum separation ----
safety_distance=0.25;
dmin = max(2*p_swarm.r_coll+safety_distance, 1.0);        % safe baseline; tune if needed
% dmin = max(2*p_swarm.r_coll, 0.5*p_swarm.d_ref);

p_swarm.Pos0 = spawn_with_min_dist_in_cube(p_swarm.P0, p_swarm.P, p_swarm.nb_agents, dmin);
p_swarm.Vel0 = p_swarm.V0 + p_swarm.V * rand(3,p_swarm.nb_agents);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Call algorithm-specific swarm parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist('SWARM_ALGORITHM','var')
    str = "param_";
    run(strcat(str, SWARM_ALGORITHM));
end

%spawner
function Pos0 = spawn_with_min_dist_in_cube(P0, P, N, dmin)
    Pos0 = zeros(3, N);

    maxTriesPerAgent = 10000;
    maxTriesGlobal   = 100;

    for g = 1:maxTriesGlobal
        % reset placement for this global attempt
        Pos0 = zeros(3, N);
        success = true;

        for i = 1:N
            placed = false;

            for k = 1:maxTriesPerAgent
                cand = P0 + P * (rand(3,1) - 0.5);

                if i == 1
                    Pos0(:, i) = cand;
                    placed = true;
                    break
                end

                d = vecnorm(Pos0(:, 1:i-1) - cand, 2, 1);
                if all(d >= dmin)
                    Pos0(:, i) = cand;
                    placed = true;
                    break
                end
            end

            if ~placed
                success = false;
                break  % break out of i-loop, restart globally
            end
        end

        if success
            return  % all agents placed successfully
        end
    end

    error("Spawn failed after %d global attempts (per-agent tries=%d). Increase P or reduce dmin.", ...
          maxTriesGlobal, maxTriesPerAgent);
end


