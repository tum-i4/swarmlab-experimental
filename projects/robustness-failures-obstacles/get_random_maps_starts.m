function [maps_sample, starts_sample, save_idx] = get_random_maps_starts(...
    path_map_file, path_starts_file, sample_size_env, sample_swarm_sizes, ...
    idx_avoid)
% Extract starting coordinates from pre-generated data.

% Load map data; rows align with number of obstacles, columns are randomly
% generated configs for that number of obstacles.
load(path_map_file, "save_maps")
[num_obs_options, num_trials] = size(save_maps.maps);

% Load starting locations
load(path_starts_file, "swarm_sizes", "start_buffer", "agent_positions", "Lx", "Ly", "Lz")

% Initialize storage
total_samples = sample_size_env * length(sample_swarm_sizes);
maps_sample = cell(total_samples, 1);  % store maps
starts_sample(total_samples) = struct('agent_positions', [], ...  % store starting locations
    'start_buffer', [], 'Lx', [], 'Ly', [], 'Lz', []);  
save_idx(total_samples) = struct('maps_row', [], ...  % store indices of selected maps and locations
    'maps_col', [], 'agent_positions_row', [], 'agent_positions_col', []);
sample_idx = 1;

% run sampling
for swarm_size_idx = 1:length(sample_swarm_sizes)
    current_swarm_size = sample_swarm_sizes(swarm_size_idx);
    
    % Find which index in the loaded data corresponds to this swarm size
    swarm_size_data_idx = find(swarm_sizes == current_swarm_size);
    if isempty(swarm_size_data_idx)
        error('Swarm size %d not found in loaded starting location data', current_swarm_size);
    end
    
    % Generate random samples for this swarm size
    for env_sample = 1:sample_size_env

        % Initialize avoidance check variables
        max_attempts = 1000;  % Maximum attempts to find unique combination
        attempt = 0;
        is_unique = false;
        
        while ~is_unique && (attempt <= max_attempts)
            % Randomly select a map (from all obstacle counts and trials)
            random_obs_idx = randi(num_obs_options);
            random_trial_idx = randi(num_trials);
            
            % Randomly select starting locations for current swarm size
            random_start_trial = randi(size(agent_positions, 2));
            
            % Check if this combination should be avoided
            if isempty(idx_avoid)
                % No avoidance needed - accept this combination
                is_unique = true;
            else
                % Check if this combination matches any in idx_avoid
                is_unique = true;  % Assume unique until proven otherwise
                for avoid_idx = 1:length(idx_avoid)
                    if idx_avoid(avoid_idx).maps_row == random_obs_idx && ...
                       idx_avoid(avoid_idx).maps_col == random_trial_idx && ...
                       idx_avoid(avoid_idx).agent_positions_row == swarm_size_data_idx && ...
                       idx_avoid(avoid_idx).agent_positions_col == random_start_trial
                        % This combination exists in idx_avoid - not unique
                        is_unique = false;
                        break;
                    end
                end
            end
            
            attempt = attempt + 1;
        end
        
        % Issue warning if max attempts reached
        if ~is_unique
            warning('Could not find unique map/start combination after %d attempts. Using last generated combination.', max_attempts);
        end
        
        % Store the selected combination
        maps_sample{sample_idx} = save_maps.maps{random_obs_idx, random_trial_idx};
        starts_sample(sample_idx).agent_positions = agent_positions{swarm_size_data_idx, random_start_trial};
        starts_sample(sample_idx).start_buffer = start_buffer;
        starts_sample(sample_idx).Lx = Lx;
        starts_sample(sample_idx).Ly = Ly;
        starts_sample(sample_idx).Lz = Lz;

        % Save randomly generated idx values for later
        save_idx(sample_idx).maps_row = random_obs_idx;
        save_idx(sample_idx).maps_col = random_trial_idx;
        save_idx(sample_idx).agent_positions_row = swarm_size_data_idx;
        save_idx(sample_idx).agent_positions_col = random_start_trial;
        
        sample_idx = sample_idx + 1;
    end
end

end