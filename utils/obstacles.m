% Static methods for handling calculations related to obstacles
classdef obstacles

    methods (Static = true)

        function [in_block, dist, unit_vect] = check_inside_block(pos_agent, x_lims, y_lims, z_lims)
            % Check if point is inside block and its distance to the block.
            % Can be run for a set of points (see format).
            % Inputs:
            %   pos_agent       Nx3 coordinates (for N points)
            %   x_lims          limits of block on x-axis (2-element vect)
            %   y_lims          limits of block on y-axis (2-element vect)
            %   z_lims          limits of block on z_axis (2-element vect)

            % Get number of positions
            N = size(pos_agent, 1);

            % Check if each point is in the block
            min_x_lims = min(x_lims);
            max_x_lims = max(x_lims);
            min_y_lims = min(y_lims);
            max_y_lims = max(y_lims);
            min_z_lims = min(z_lims);
            max_z_lims = max(z_lims);
            in_block = (pos_agent(:,1) > min_x_lims & pos_agent(:,1) < max_x_lims) & ...
                (pos_agent(:,2) > min_y_lims & pos_agent(:,2) < max_y_lims) & ...
                (pos_agent(:,3) > min_z_lims & pos_agent(:,3) < max_z_lims);

            % Any points not in the block
            if any(~in_block)

                % Get closest point on the block
                min_x_lims = ones(N,1) * min_x_lims;
                max_x_lims = ones(N,1) * max_x_lims;
                min_y_lims = ones(N,1) * min_y_lims;
                max_y_lims = ones(N,1) * max_y_lims;
                min_z_lims = ones(N,1) * min_z_lims;
                max_z_lims = ones(N,1) * max_z_lims;
                cx = max([min_x_lims, ...
                    min([pos_agent(:,1), max_x_lims], [], 2)], [], 2);
                cy = max([min_y_lims, ...
                    min([pos_agent(:,2), max_y_lims], [], 2)], [], 2);
                cz = max([min_z_lims, ...
                    min([pos_agent(:,3), max_z_lims], [], 2)], [], 2);

                % Calculate vector from point to closest point
                Vx = cx - pos_agent(:,1);
                Vy = cy - pos_agent(:,2);
                Vz = cz - pos_agent(:,3);

                % Normalize the vector (calculate distance)
                dist = sqrt(Vx.^2 + Vy.^2 + Vz.^2);
                unit_vect = [Vx./dist, Vy./dist, Vz./dist];

                % If mix of points in and outside the block, reset 
                % distance and unit vector for the points in the block
                if any(in_block)
                    dist(in_block) = 0;
                    unit_vect(in_block,:) = ...
                        repmat([nan, nan, nan], sum(in_block), 1);
                end

            else

                % All points in block
                dist = zeros(N, 1);
                unit_vect = nan(N, 3);

            end

        end
        

        function [limits, faces, centers, normals] = get_block_properties(x, y, z, x_width, y_width, z_width)
            % Get properties of block. Note this is limited to blocks with faces that
            % are in the same planes as the x-y, x-z, and y-z planes. To make rotated
            % blocks, apply a rotation to the vertices, centers, and normals.
            % Output information in consistent order.
            %
            % Inputs:
            %   x, y, z    1xn vectors of block centers for n blocks.
            %   x_width, y_width, z_width
            %              1xn vectors of block widths for n blocks.
            % Outputs:
            %   limits      6xn matrix for n blocks, where each row is: x_min, x_max,
            %               y_min, y_max, z_min, z_max.
            %   faces       1xn cell array; in each cell is a 4x18 matrix of 8
            %               vertices 3-d coordinates. For n blocks.
            %   centers     1xn cell array; in each cell is a 6x3 matrix of 3-d
            %               coordinates for 6 sides of the block.
            %   normals     1xn cell array; in each cell is a 6x3 matrix of vectors
            %               that represent the normal vector of each of the 6 sides.

            % Initialize outputs
            num_blocks = length(x);
            limits = nan(6, num_blocks);
            faces = cell(1, num_blocks);
            centers = cell(1, num_blocks);
            normals = cell(1, num_blocks);

            % Iterate through each block
            for i = 1:num_blocks

                % Get limits
                x_min = x(i) - (1/2)*x_width(i);
                x_max = x(i) + (1/2)*x_width(i);
                y_min = y(i) - (1/2)*y_width(i);
                y_max = y(i) + (1/2)*y_width(i);
                z_min = z(i) - (1/2)*z_width(i);
                z_max = z(i) + (1/2)*z_width(i);

                % Put into output array
                limits(:,i) = [x_min; x_max; y_min; y_max; z_min; z_max];

                % % Vertices
                % vertices{i} = [x_min y_min z_min;
                %     x_max y_min z_min;
                %     x_max y_max z_min;
                %     x_min y_max z_min;
                %     x_min y_min z_max;
                %     x_max y_min z_max;
                %     x_max y_max z_max;
                %     x_min y_max z_max]';

                % Faces (coordinates in right hand rule order)
                face_bottom = [x_min y_min z_min;
                    x_min y_max z_min;
                    x_max y_max z_min;
                    x_max y_min z_min];
                face_top = [x_min y_min z_max;
                    x_max y_min z_max;
                    x_max y_max z_max;
                    x_min y_max z_max];
                face_left = [x_min y_min z_min;
                    x_min y_min z_max;
                    x_min y_max z_max;
                    x_min y_max z_min];
                face_right = [x_max y_min z_min;
                    x_max y_max z_min;
                    x_max y_max z_max;
                    x_max y_min z_max];
                face_front = [x_min y_min z_min;
                    x_max y_min z_min;
                    x_max y_min z_max;
                    x_min y_min z_max];
                face_back = [x_min y_max z_min;
                    x_max y_max z_min;
                    x_max y_max z_max;
                    x_min y_max z_max];
                faces{i} = [face_bottom, face_top, face_left, face_right, face_front, face_back];


                % Centers
                centers{i} = [mean([x_min x_max]) mean([y_min y_max]) z_min;    % Bottom
                    mean([x_min x_max]) mean([y_min y_max]) z_max;          % Top
                    x_min mean([y_min y_max]) mean([z_min z_max]);          % Left
                    x_max mean([y_min y_max]) mean([z_min z_max]);          % Right
                    mean([x_min x_max]) y_min mean([z_min z_max]);          % Front
                    mean([x_min x_max]) y_max mean([z_min z_max])];        % Back

                % Normals
                normals{i} = [0 0 -1;   % Bottom
                    0 0 1;      % Top
                    -1 0 0;     % Left
                    1 0 0;      % Right
                    0 -1 0;     % Front
                    0 1 0];    % Back

            end

        end

        function [dist] = get_dist_to_block_face(pos_agent, face_point, face_norm_vector)
            % Get shortest distance between face and drone. This operates 
            % on only one face.
            % Inputs:
            %   pos_agent       1x3 coordinates
            %   r_agent         scalar of agent keep-away radius
            %   face_points     1x3 matrix of one of the face's points, 
            %                   in (x, y, z)
            %   face_norm_vector    1x3 normal vector of block face

            % Need to calculate the closest distance between plane and
            % point
            A = face_norm_vector(1);
            B = face_norm_vector(2);
            C = face_norm_vector(3);
            D = -(A*face_point(1) + B*face_point(2) + C*face_point(3));

            % Calculate closest distance between point and face
            dist = abs(A*pos_agent(1) + B*pos_agent(2) + C*pos_agent(3) + D) / ...
                sqrt(A^2 + B^2 + C^2);

        end

        function [unit_vec, dist] = get_single_face_influence_vect(pos_agent, vel_agent, face_points, face_norm_vector, face_center_point)
            % Get influence vector of face on the drone. If the drone is
            % directly facing the face, the influence vector will try to
            % push the drone out and away. If the drone is not directly
            % facing the face, the influence vector will simply be the
            % normal vector.
            % Also outputs distance.
            % Inputs:
            %   pos_agent       1x3 coordinates
            %   vel_agent       1x3 vector of velocity
            %   face_points     4x3 matrix of 3D coordinates defining one
            %                   face of the block (4 points).
            %   face_norm_vector    1x3 normal vector of block face
            %   face_center_point   1x3 coordinates of the center of the face

            % Need to calculate the closest distance between plane and
            % point
            A = face_norm_vector(1);
            B = face_norm_vector(2);
            C = face_norm_vector(3);
            D = -(A*face_points(1,1) + B*face_points(1,2) + C*face_points(1,3));

            % Calculate closest distance between point and face
            dist = abs(A*pos_agent(1) + B*pos_agent(2) + C*pos_agent(3) + D) / ...
                sqrt(A^2 + B^2 + C^2);

            % Check if drone velocity vector has positive dot product to
            % opposite of face normal vector (then drone is flying at it)
            if dot(vel_agent, face_norm_vector*-1) > 0

                % Check if drone velocity vector intersects face
                ndotd = dot(face_norm_vector, vel_agent);
                t = dot(face_norm_vector, (face_points(1,:) - pos_agent)) / ndotd;
                intersectPt = pos_agent + t * vel_agent;

                % Define edges of the rectangle
                edge1 = face_points(2,:) - face_points(1,:);
                edge2 = face_points(4,:) - face_points(1,:);

                % Project intersection point onto coordinate system defined
                % by edges
                AP = intersectPt - face_points(1,:);

                % Solve for coordinates (u,v) in the local plane
                % coordinates
                u = dot(AP, edge1) / dot(edge1, edge1);
                v = dot(AP, edge2) / dot(edge2, edge2);

                % Check if velocity vector is pointing at the face by
                % checking if (u, v) lies within the relative coordinate  
                % ranges [0, 1] x [0, 1]
                if u >= 0 && u <= 1 && v >= 0 && v <= 1
                    
                    % Project drone position onto plane
                    temp_term = dist / sqrt(A^2 + B^2 + C^2);
                    plane_x = pos_agent(1) - A * temp_term;
                    plane_y = pos_agent(2) - B * temp_term;
                    plane_z = pos_agent(3) - C * temp_term;
    
                    % Iterate through the face points to figure out which two points the projected
                    % drone position is closest to
                    sep_dist = vecnorm([plane_x plane_y plane_z] - face_points, 2, 2);
                    [~, sort_ind] = sort(sep_dist);
                    closest_points = face_points(sort_ind(1:2),:);
    
                    % Get unit vector in the direction of the line
                    n_line = closest_points(2,:) - closest_points(1,:);
                    n_line = n_line / norm(n_line);
    
                    % Calculate normal vector from point to line
                    a = closest_points(1,:);
                    pt_ln_vec = (a-[plane_x plane_y plane_z]) - dot(a - [plane_x plane_y plane_z], n_line) * n_line;
                    pt_ln_dist = norm(pt_ln_vec);
                    pt_ln_vec = pt_ln_vec / pt_ln_dist; % make it a unit vector
    
                    % Based on the distance of the point from the line, adjust the unit vector's
                    % in-plane and out-of-plane components
                    max_dist = norm(a - face_center_point) - dot(a - face_center_point, n_line) * n_line;
                    max_dist = norm(max_dist);
                    out_over_norm = pt_ln_dist / max_dist;
                    unit_vec = pt_ln_vec*(out_over_norm) + face_norm_vector*(1-out_over_norm);
                    unit_vec = unit_vec/norm(unit_vec);

                else

                    % Vector is just norm of face
                    unit_vec = face_norm_vector;

                end

            else

                % Vector is just norm of face
                unit_vec = face_norm_vector;

            end

        end

        function [unit_vec, dist] = get_block_edge_to_point(pos_agent, face1_points, face2_points)
            % Get unit vector and distance of the shortest line between a
            % point and the edge connecting two adjacent faces of the
            % block.
            % Inputs:
            %   pos_agent      1x3 coordinates
            %   face1_points    4x3 matrix of 3D coordinates defining one
            %                   face of the block (4 points).
            %   face2_points    Same as face1_points but for an adjacent
            %                   face.

            % Check which two points are shared by both faces.
            shared_points = intersect(face1_points, face2_points, 'rows');

            % Unit vector in the direction of the line
            n_line = shared_points(2,:) - shared_points(1,:);
            n_line = n_line / norm(n_line);

            % Calculate normal vector from line to point
            a = shared_points(1,:);
            unit_vec = -((a-pos_agent) - dot(a - pos_agent, n_line) * n_line);
            dist = norm(unit_vec);
            unit_vec = unit_vec / dist; % make it a unit vector

        end

        function [unit_vec, dist] = get_block_corner_to_point(pos_agent, face1_points, face2_points, face3_points)
            % Get unit vector and distance of the line between a
            % point and the corner connecting three adjacent faces of the
            % block.
            % Inputs:
            %   pos_agent      1x3 coordinates
            %   face1_points    4x3 matrix of 3D coordinates defining one
            %                   face of the block (4 points).
            %   face2/3_points  Same as face1_points but for an adjacent
            %                   face.

            % Check which point is shared by all three faces.
            shared_point = intersect(intersect(face1_points, face2_points, 'rows'), face3_points, 'rows');

            % Vector from corner to drone
            unit_vec = pos_agent - shared_point;
            dist = norm(unit_vec);
            unit_vec = unit_vec / dist;

        end


    end

end

