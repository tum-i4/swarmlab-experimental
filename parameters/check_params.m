function [p_sim, p_battery, p_physics, p_drone, map, p_swarm] = ...
    check_params(p_sim, p_battery, p_physics, p_drone, map, p_swarm)
%CHECK_PARAMS checks config parameters set in parameter scripts and sets
% defaults where needed. New config variables should be accounted for here.

% Swarm goal type flag: set default of false if not included
check_fields = {'is_active_goal', 'is_active_migration'};
for i = length(check_fields)
    if ~isfield(p_swarm, check_fields{i})
        p_swarm.(check_fields{i}) = false;
    end
end

% Obstacle & Arena calculation flags: set default to false if not included
check_fields = {'is_active_arena', 'is_active_spheres', 'is_active_cyl', ...
    'is_active_blocks'};
for i = length(check_fields)
    if ~isfield(p_swarm, check_fields{i})
        p_swarm.(check_fields{i}) = false;
    end
end

% If cylinders active, check that vector of widths exists; otherwise assume
% that "building_width" exists and get it 
if p_swarm.is_active_cyl

    if ~isfield(map, 'buildings_width')
        map.buildings_width = repmat(map.building_width, size(map.buildings_north));
    end

end

end

