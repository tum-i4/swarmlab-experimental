function fig_handle = draw_obstacles(fig_handle, map, p_swarm)
% DRAW_OBSTACLES - draw cylinder and sphere obstacles in figures

gray_shade = 0.8;

% Draw buildings/cylinder obstacles if active
if p_swarm.is_active_cyl

    NB_EDGES  = 16;

    nb_buildings = length(map.buildings_north);
    for i = 1:nb_buildings

        [X,Y,Z] = cylinder(map.buildings_width(i)/2, NB_EDGES);
        Z = map.max_height * Z;

        Xtrasl = X + map.buildings_north(i);
        Ytrasl = Y + map.buildings_east(i);
        C = repmat(gray_shade*ones(size(Xtrasl)),1,1,3);
        % surf(Ytrasl, Xtrasl, Z, C,'LineWidth',0.5);
        surf(Ytrasl, Xtrasl, Z, C,'LineWidth',0.5, 'FaceAlpha', 0.5); % Make semi-transparent
        hold on;

        % Add some text to each obstacle to label them
        text(Ytrasl(1,1) - map.buildings_width(i)/3, ...
            Xtrasl(1,1) - map.buildings_width(i)/2, ...
            Z(2,2) + 10.0, ...
            ['Cy' num2str(i)], 'FontSize', 8);
    end

end

% Draw spheres/spherical obstacles if active
if p_swarm.is_active_spheres

    [sphX, sphY, sphZ] = sphere;
    sphC = repmat(gray_shade * ones(size(sphX)), 1, 1, 3);
    for jj = 1:map.n_spheres
        Xtrasl2 = sphX * map.spheres_r(jj) + map.spheres_north(jj);
        Ytrasl2 = sphY * map.spheres_r(jj) + map.spheres_east(jj);
        Ztrasl2 = sphZ * map.spheres_r(jj) - map.spheres_down(jj);
        % surf(Ytrasl2, Xtrasl2, Ztrasl2, sphC, 'LineWidth', 0.5);
        surf(Ytrasl2, Xtrasl2, Ztrasl2, sphC, 'LineWidth', 0.5,...
            'FaceAlpha', 0.5); % Make semi-transparent
        hold on

        % Add some text to each obstacle to label them
        adjust_scale = 1;
        text(map.spheres_east(jj) + adjust_scale*map.spheres_r(jj), ...
            map.spheres_north(jj) + adjust_scale*map.spheres_r(jj), ...
            -map.spheres_down(jj), ...
            ['Sp' num2str(jj)], 'FontSize', 8);
    end

end

% Draw block obstacles if active
if p_swarm.is_active_blocks

    nb_blocks = size(map.blocks_limits, 2); % Number of blocks
    for i = 1:nb_blocks

        % Map over from NED to typical figure XYZ
        % x_min = map.blocks(1, i);
        % x_max = map.blocks(2, i);
        % y_min = map.blocks(3, i);
        % y_max = map.blocks(4, i);
        % z_min = map.blocks(5, i);
        % z_max = map.blocks(6, i);
        x_min = map.blocks_limits(3, i); % east-min
        x_max = map.blocks_limits(4, i); % east-max
        y_min = map.blocks_limits(1, i); % north-min
        y_max = map.blocks_limits(2, i); % north-max
        z_min = map.blocks_limits(6, i) * -1; % down-max * -1
        z_max = map.blocks_limits(5, i) * -1; % down-min * -1

        % Define vertices of the block
        vertices = [x_min y_min z_min;
            x_max y_min z_min;
            x_max y_max z_min;
            x_min y_max z_min;
            x_min y_min z_max;
            x_max y_min z_max;
            x_max y_max z_max;
            x_min y_max z_max];

        % Define faces of the block (6 faces)
        faces = [1 2 3 4;  % Bottom face (z = z_min)
            5 6 7 8;  % Top face (z = z_max)
            1 2 6 5;  % Side face (y = y_min)
            2 3 7 6;  % Side face (x = x_max)
            3 4 8 7;  % Side face (y = y_max)
            4 1 5 8]; % Side face (x = x_min)

        % Draw the block using the patch function
        patch('Vertices', vertices, 'Faces', faces, ...
            'FaceColor', gray_shade * [1 1 1], 'EdgeColor', 'k', 'LineWidth', 0.5);
        hold on;

        % Add some text to label each block
        text((x_min + x_max) / 2, (y_min + y_max) / 2, z_max + 10.0, ...
            ['Bk' num2str(i)], 'FontSize', 8);
    end

end

% % Set figure limits and axes scaling
% if (p_swarm.is_active_cyl || p_swarm.is_active_spheres)
% 
%     map_width = map.width;
%     axes_lim = [-map_width/5 + map.bl_corner_east, ... % x_min
%         map_width + map_width/5 + map.bl_corner_east, ... % x_max
%         -map_width/4 + map.bl_corner_north, ... % y_min
%         map_width + map_width/4 + map.bl_corner_north, ... % y_max
%         0, ... % z_min
%         1.2*map.max_height]; % z_max
%     axis equal
%     axis(axes_lim);
%     view(0,90);
%     % axis equal;
% 
% end

end
