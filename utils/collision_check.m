function [coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
    pos_ned_history, p_swarm, coll_dist)
% Function checks for collisions based on UAV position information and
% scenario configuration.
% If a collision occurs, only the first time the
% collision is detected should be used; since collision effects are not
% % simulated, data after the collision should not be considered valid.
% If a collision does not occur, the closest distance is recorded instead.
%
% Inputs:
%   pos_ned_history = n x (3*m) matrix with NED positions for m drones
%   p_swarm         = struct with scenario configuration information
%   coll_dist       = collision distance threshold; set [] to use
%                       p_swarm.r_coll as the threshold.
% Outputs:
%   All outputs include a "occur" flag to indicate if that type of
%   collision occurred.
%   coll_result     = struct with "occur" flag to show if any of the
%                       possible collision types occurred; fields "type",
%                       "drone_num", "index", and "dist" provide
%                       information on the first collision that occurred.
%   coll_obs        = struct with time index and drone number of first
%                       collision between a drone and an obstacle.
%   coll_drones     = struct with time index and drone numbers of first
%                       collision between two (or more) drones.
%   coll_walls      = struct with time index and drone number of first
%                       collision between a drone and a geofence wall.

% Set collision distance threshold based on input
if isempty(coll_dist)
    coll_dist = p_swarm.r_coll;
end

% Initialize outputs
% Final result
coll_result.occur = false;
coll_result.type = nan;
coll_result.drone_num = nan;
coll_result.index = nan;
coll_result.dist = nan;
% Obstacle-collision only
coll_obs = coll_result;
coll_obs.type = 'obstacle-drone';
% Drone-drone only
coll_drones = coll_result;
coll_drones.type = 'drone-drone';
% Drone-wall only
coll_walls = coll_result;
coll_walls.type = 'boundary-drone';

% Number of drones & time steps
num_drones = size(pos_ned_history, 2) / 3;

%% Check drone-obstacle collisions

% Spherical obstacles
if p_swarm.is_active_spheres

    % Iterate through sphere obstacles
    for i1 = 1:p_swarm.n_spheres

        % Get sphere coordinates and radius
        temp_center = p_swarm.spheres(1:3, i1);
        temp_center = temp_center(:)'; % ensure it's a row vector
        temp_radius = p_swarm.spheres(4, i1);

        % Iterate through drones
        for j1 = 1:num_drones

            % Index for pos matrix
            temp_ind = (3*(j1-1))+1;

            % Calculate distance between drone positions and obstacle
            % (3D for sphere)
            temp_dist = pos_ned_history(:,temp_ind:temp_ind+2) - temp_center;
            temp_dist = sqrt(sum(temp_dist.^2, 2)) - temp_radius; % subtract obstacle radius from distance

            % Check if collision occurred; if so, record first one if it is
            % earlier than existing
            coll_obs = check_and_record(coll_obs, temp_dist, coll_dist, j1);

        end

    end

end

% Cylindrical obstacles
if p_swarm.is_active_cyl

    % Iterate through cylinder obstacles
    for m1 = 1:p_swarm.n_cyl

        % Get cylinder coordinates and radius; note, we only use x-y
        % coordinates for cylinders
        temp_center = p_swarm.cylinders(1:2, m1);
        temp_center = temp_center(:)'; % ensure it's a row vector
        temp_radius = p_swarm.cylinders(3, m1);

        % Iterate through drones
        for n1 = 1:num_drones

            % Index for pos matrix
            temp_ind = (3*(n1-1))+1;

            % Calculate distance between drone positions and obstacle
            % (2D for cylinder)
            temp_dist = pos_ned_history(:, temp_ind:temp_ind+1) - temp_center;
            temp_dist = sqrt(sum(temp_dist.^2, 2)) - temp_radius; % subtract obstacle radius from distance

            % Check if collision occurred; if so, record first one if it is
            % earlier than existing
            coll_obs = check_and_record(coll_obs, temp_dist, coll_dist, n1);

        end

    end

end

% Block obstacles
if isfield(p_swarm, 'is_active_blocks') && p_swarm.is_active_blocks

    % Iterate through blocks
    for r1 = 1:p_swarm.n_blocks

        % Iterate through drones
        for s1 = 1:num_drones

            % Index for pos matrix
            temp_ind = (3*(s1-1))+1;

            % Calculate distance between drone positions and obstacle
            [~, temp_dist, ~] = obstacles.check_inside_block(...
                pos_ned_history(:, temp_ind:temp_ind+2), ...
                p_swarm.blocks_limits(1:2, r1), ...
                p_swarm.blocks_limits(3:4, r1), ...
                p_swarm.blocks_limits(5:6, r1));

            % Check if collision occurred; if so, record first one if it is
            % earlier than existing
            coll_obs = check_and_record(coll_obs, temp_dist, coll_dist, s1);

        end

    end

end

%% Check drone-drone collisions

% Iterate through drones
for i2 = 1:num_drones

    for j2 = i2:num_drones

        % Skip self
        if i2 == j2
            continue
        end

        % Indices for drone positions
        temp_ind_1 = (3*(i2-1))+1;
        temp_ind_2 = (3*(j2-1))+1;

        % Calculate distance between drone positions and obstacles
        temp_dist = pos_ned_history(:, temp_ind_1:temp_ind_1+2) - ...
            pos_ned_history(:, temp_ind_2:temp_ind_2+2);
        % one drone radius accounted for here, the other in
        % check_and_record
        temp_dist = sqrt(sum(temp_dist.^2, 2)) - coll_dist;

        % Check if collision occurred; if so, record first one if it is
        % earlier than existing
        coll_drones = check_and_record(coll_drones, temp_dist, coll_dist, i2, j2);

    end

end

%% Check drone-wall collisions

% Only run if arena/fences are active
if p_swarm.is_active_arena

    % Iterate through drones
    for i3 = 1:num_drones

        % Iterate through axes (X-Y-Z / N-E-D)
        for j3 = 1:3

            % Index for drone position
            temp_ind = (3*(i3-1))+1;

            % Min and max walls for this axis
            temp_min = min(p_swarm.x_arena(j3,:));
            temp_max = max(p_swarm.x_arena(j3,:));

            % Calculate separation from drone position and wall along this axis
            % Min side
            temp_dist = pos_ned_history(:, temp_ind+(j3-1)) - temp_min;
            coll_walls = check_and_record(coll_walls, temp_dist, coll_dist, i3);
            % Max side
            temp_dist = temp_max - pos_ned_history(:, temp_ind+(j3-1));
            coll_walls = check_and_record(coll_walls, temp_dist, coll_dist, i3);

        end

    end

end

%% At very end: put first collision into result summary

% Put structs into cell to easily iterate through
temp_cell = {coll_obs, coll_drones, coll_walls};

% Check collision detection: final result is collision that occurs and is
% earliest.
for i4 = 1:length(temp_cell)

    if temp_cell{i4}.occur
        if ~coll_result.occur || ...
                (coll_result.occur && (temp_cell{i4}.index < coll_result.index))
            coll_result = temp_cell{i4};
        end
    end

end

end

%% Local functions

function [coll_struct] = check_and_record(coll_struct, check_dist, coll_dist, drone_num, varargin)
% Update coll_* structure based on whether or not a collision has
% occurred with this drone and obstacle pairing

% Subtract collision distance threshold from distances
check_dist = check_dist - coll_dist;

% Check if collision occurs
check_coll = check_dist <= 0;

% If optional input provided, assume it is a second drone
if ~isempty(varargin)
    drone_num = [drone_num, varargin{1}];
end

% If collision does occur
if any(check_coll)

    % Find first instance of collision
    index_first = find(check_coll, 1, 'first');

    % Update coll_* struct
    if (coll_struct.occur == false) || ...
            ((coll_struct.occur == true) && (index_first < coll_struct.index))
        coll_struct.occur = true;
        coll_struct.drone_num = drone_num;
        coll_struct.index = index_first;
        coll_struct.dist = check_dist(index_first);
    end

elseif  coll_struct.occur == false
    % If an earlier collision has not already been detected, update
    % information with closest approach.

    % Find closest that drone gets to obstacle and save that if it is less
    % than what is already saved.
    min_dist = min(check_dist);
    min_dist_ind = find(check_dist == min_dist, 1, 'first');
    if isnan(coll_struct.dist) || (min_dist < coll_struct.dist)
        coll_struct.drone_num = drone_num;
        coll_struct.index = min_dist_ind;
        coll_struct.dist = min_dist;
    end

end

end