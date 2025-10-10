classdef RunSetOfTests < matlab.unittest.TestCase
    %RUNSETOFTESTS This uses the MATLAB testing framework to run tests,
    %including a selection of functions from this directory.
    %   Detailed explanation goes here

    properties (Access = private)
        originalPath
        originalDir
    end

    methods (TestClassSetup)
        function setupClass(testCase)
            % Save original state
            testCase.originalPath = path;
            testCase.originalDir = pwd;

            % Reset to clean path state
            restoredefaultpath;

            % Navigate to project root (parent of \tests directory)
            homeDir = fullfile(fileparts(mfilename('fullpath')), '..');
            cd(homeDir);

            % Add all project directories to path
            addpath(genpath(pwd));

            fprintf('Test environment initialized:\n');
            fprintf('  - Working directory: %s\n', pwd);
            fprintf('  - Path reset and project directories added\n');
        end
    end

    methods (TestClassTeardown)
        function teardownClass(testCase)
            % Restore original state
            path(testCase.originalPath);
            cd(testCase.originalDir);

            fprintf('Original directory paths restored\n');
        end
    end

    methods (Test)
        function testRegressionVasarhelyiRefactor(testCase)
            % Tests any changes that are made to the vasarhelyi swarm calcs
            % by running the original scenario with both the original and
            % current versions of vasarhelyi and comparing the results.
            %
            % Note that TestClassSetup methods should have already set up 
            % relevant directory paths.

            % First the point mass version
            point_mass_out = test_example_vasarhelyi('vasarhelyi', 'point_mass');
            point_mass_out_orig = test_example_vasarhelyi('vasarhelyi_original', 'point_mass');

            % Check if results are equal
            for data_field = fields(point_mass_out)'
                testCase.verifyEqual(point_mass_out.(data_field{1}), ...
                    point_mass_out_orig.(data_field{1}))
            end

            % Plot for visual check
            plot_case(point_mass_out_orig.pos_ned_history, ...
                point_mass_out_orig.p_swarm, point_mass_out_orig.map);

            % Also check if velocities in XYZ and NED are the same 
            % (should be for point mass model)
            testCase.verifyEqual(point_mass_out.vel_xyz_history, ...
                point_mass_out.vel_ned_history, 'AbsTol', 1e-10)

            % Next run the the quadcopter version
            quadcopter_out = test_example_vasarhelyi('vasarhelyi', 'quadcopter');
            quadcopter_out_orig = test_example_vasarhelyi('vasarhelyi_original', 'quadcopter');

            % Check if results are equal
            for data_field = fields(quadcopter_out)'
                testCase.verifyEqual(quadcopter_out.(data_field{1}), ...
                    quadcopter_out_orig.(data_field{1}))
            end

            % Plot for visual check
            plot_case(quadcopter_out_orig.pos_ned_history, ...
                quadcopter_out_orig.p_swarm, quadcopter_out_orig.map);

            % Check if speeds calculated from velocities from different
            % reference frames are equal.
            num_agents = size(quadcopter_out.vel_xyz_history, 2) / 5;
            for i = 1:num_agents
                speed_ned(:,i) = vecnorm(quadcopter_out.vel_ned_history(:,(3*(i-1)+1):(3*(i-1)+3)),2,2);
                speed_xyz(:,i) = vecnorm(quadcopter_out.vel_xyz_history(:,(3*(i-1)+1):(3*(i-1)+3)),2,2);
            end
            testCase.verifyEqual(speed_xyz, speed_ned, 'AbsTol', 1e-10)

        end

        function testEndToEnd(testCase)
            % Runs end-to-end test to verify that no errors occur.
            %
            % Note that TestClassSetup methods should have already set up 
            % relevant directory paths.

            % Set up scenario where a collision should occur
            % (check param_sim; collision should occur if dt is 0.01 or 0.1)
            in_x_centroid = 0; in_y_centroid = 190.309;
            in_x_vec = [211.8; 120.44; 147.496];
            in_y_vec = [50.6859; 74.4602; 193.871];
            in_num_drone = 5;

            % Run simulation where obstacle placement can be varied
            [out_pos, out_vel, out_time, out_p_swarm, out_map, out_alg_conn] = ...
                obstacle_test(...
                in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone);

            % Plot result
            plot_case(out_pos, out_p_swarm, out_map)

            % Run collision check again to show its outputs (this is also run within
            % plot_case)
            [coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
                out_pos, out_p_swarm, out_p_swarm.r_coll);

            % Also try a run without a collision
            in_x_centroid = 0; in_y_centroid = 180;
            in_x_vec = [200; 120; 140];
            in_y_vec = [50; 70; 190];
            in_num_drone = 5;

            [out_pos, out_vel, out_time, out_p_swarm, out_map, out_alg_conn] = ...
                obstacle_test(...
                in_x_centroid, in_y_centroid, in_x_vec, in_y_vec, in_num_drone);

            plot_case(out_pos, out_p_swarm, out_map)

        end

        function testEndToTendParallelProcessing(testCase)
            % Runs end-to-end test to verify that no errors occur. Uses
            % paralle processing.
            %
            % Note that TestClassSetup methods should have already set up 
            % relevant directory paths.

            % Set up vectors and matrices of inputs
            num_indiv = 20;
            in_x_centroid = zeros(num_indiv, 1);
            in_y_centroid = 190.309.*ones(num_indiv, 1);
            in_x_coords = repmat([211.8, 120.44, 147.496], num_indiv, 1);
            in_y_coords = repmat([50.6859, 74.4602, 193.871], num_indiv, 1);
            in_num_drone = 5.*ones(num_indiv, 1);

            % Set up output
            c1 = cell(num_indiv, 4);

            % Parallel loops
            tic
            [cell_coll_result, cell_coll_obs, cell_coll_drones, cell_coll_walls] = ...
                wrapper_obstacle_test_parallel(...
                in_x_centroid, in_y_centroid, in_x_coords, in_y_coords, in_num_drone);
            fprintf('Parallel processing of %d individuals.\n', num_indiv)
            toc

            % Serial for comparison
            % Set up output
            c2 = cell(num_indiv, 4);

            % Serial loops
            tic
            for i = 1:num_indiv

                % Run simulation where obstacle placement can be varied
                [out_pos, ~, ~, out_p_swarm, ~, ~] = ...
                    obstacle_test(...
                    in_x_centroid(i), in_y_centroid(i), in_x_coords(i,:), ...
                    in_y_coords(i,:), in_num_drone(i));

                % Run collision check
                [coll_result, coll_obs, coll_drones, coll_walls] = collision_check(...
                    out_pos, out_p_swarm, out_p_swarm.r_coll);

                c2(i,:) = {coll_result, coll_obs, coll_drones, coll_walls};

            end
            fprintf('Serial processing of %d individuals.\n', num_indiv)
            toc

        end
    end

end