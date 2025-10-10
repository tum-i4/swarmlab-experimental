function positions = place_spheres_informed(Lx, Ly, Lz, r_coll, num_agents)
    % Parameters
    box_size = [Lx, Ly, Lz]; % Box dimensions [Lx, Ly, Lz]
    radius = r_coll; % Sphere radius
    num_spheres = num_agents; % Total number of spheres to place

    % Bounds of the box
    bounds = [-box_size / 2; box_size / 2];

    % Initialize sphere positions
    positions = zeros(num_spheres, 3);
    positions(1, :) = [0, 0, 0];
    % positions(1, :) = generate_random_position(bounds); % Place the first sphere

    for i = 2:num_spheres
        positions(i, :) = place_touching_sphere(positions(1:i-1, :), radius, bounds);
    end

    % Visualization
    % visualize_spheres(bounds, positions, radius);
end

function position = place_touching_sphere(existing_positions, radius, bounds)
    max_attempts = 1000; % Limit attempts to avoid infinite loops
    for attempt = 1:max_attempts
        % Select a random existing sphere
        base_sphere = existing_positions(randi(size(existing_positions, 1)), :);

        % Generate a random direction vector
        direction = randn(1, 3);
        direction = direction / norm(direction); % Normalize to unit vector

        % Place the new sphere touching the base sphere
        candidate = base_sphere + direction * (2 * radius);

        % Validate the candidate position
        if is_valid_position(candidate, existing_positions, radius, bounds)
            position = candidate;
            return;
        end
    end

    error('Failed to place a touching sphere after %d attempts', max_attempts);
end

function valid = is_valid_position(candidate, existing_positions, radius, bounds)
    % Check if the candidate sphere is within bounds
    valid = all(candidate - radius >= bounds(1, :)) && ...
            all(candidate + radius <= bounds(2, :));
    if ~valid, return; end

    % Check for overlap with existing spheres
    if ~isempty(existing_positions)
        distances = vecnorm(existing_positions - candidate, 2, 2);
        valid = all(distances >= 2 * radius | distances <= 1e-6); % Touching allowed
    end
end

function visualize_spheres(bounds, positions, radius)
    % Visualization of the box and spheres
    figure;
    hold on;

    % Plot the box
    plot_box(bounds);

    % Plot each sphere
    [x, y, z] = sphere(20); % Generate unit sphere
    for i = 1:size(positions, 1)
        surf(radius * x + positions(i, 1), ...
             radius * y + positions(i, 2), ...
             radius * z + positions(i, 3), ...
             'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end

    axis equal;
    hold off;
end

function plot_box(bounds)
    % Draws a wireframe box
    corners = combvec(bounds(:, 1)', bounds(:, 2)', bounds(:, 3)')';
    edges = nchoosek(1:8, 2);
    for e = 1:size(edges, 1)
        corner1 = corners(edges(e, 1), :);
        corner2 = corners(edges(e, 2), :);
        if sum(abs(corner1 - corner2) > 0) == 1 % Only draw edges
            plot3([corner1(1), corner2(1)], [corner1(2), corner2(2)], [corner1(3), corner2(3)], 'k-');
        end
    end
end
