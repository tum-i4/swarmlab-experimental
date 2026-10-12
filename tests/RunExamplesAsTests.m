classdef RunExamplesAsTests < matlab.unittest.TestCase

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
        % Test methods

        function testPointMassSims(testCase)
            % Flag to run visualizations - set to false for this test script
            flag_run_viz_post = false;
            flag_run_video_post = false;

            % Set DRONE_TYPE to 'point_mass' or 'quadcopter'
            set_drone_type = 'point_mass';

            % example 1
            run(fullfile('examples','examples_new','ex1_try_faster_run.m'))

            % example 2
            run(fullfile('examples','examples_new','ex2_adjust_cyl_obstacles.m'))

            % example 3
            run(fullfile('examples','examples_new','ex3_add_spheres_obstacles.m'))

            % example 4
            run(fullfile('examples','examples_new','ex4_add_walls.m'))

            % example 5
            run(fullfile('examples','examples_new','ex5_single_UAV.m'))

            % example 6
            run(fullfile('examples','examples_new','ex6_just_sphere_obstacles.m'))

            % example 7
            run(fullfile('examples','examples_new','ex7_active_goal.m'))

            % example 8
            run(fullfile('examples','examples_new','ex8_active_goal_with_dropouts.m'))

            % May want to add checks in the future.
        end

        function testPointMassSimsWithViz(testCase)
            % Flag to run visualizations - set to TRUE for this test script
            flag_run_viz_post = true;
            flag_run_video_post = false;

            % Set DRONE_TYPE to 'point_mass' or 'quadcopter'
            set_drone_type = 'point_mass';

            % example 1
            run(fullfile('examples','examples_new','ex1_try_faster_run.m'))

            % example 2
            run(fullfile('examples','examples_new','ex2_adjust_cyl_obstacles.m'))

            % example 3
            run(fullfile('examples','examples_new','ex3_add_spheres_obstacles.m'))

            % example 4
            run(fullfile('examples','examples_new','ex4_add_walls.m'))

            % example 5
            run(fullfile('examples','examples_new','ex5_single_UAV.m'))

            % example 6
            run(fullfile('examples','examples_new','ex6_just_sphere_obstacles.m'))

            % example 7
            run(fullfile('examples','examples_new','ex7_active_goal.m'))

            % example 8
            run(fullfile('examples','examples_new','ex8_active_goal_with_dropouts.m'))

            % May want to add checks in the future.
        end

        function testQuadcopterSimsWithViz(testCase)
            % Flag to run visualizations - set to false for this test script
            flag_run_viz_post = true;
            flag_run_video_post = false;

            % Set DRONE_TYPE to 'point_mass' or 'quadcopter'
            set_drone_type = 'quadcopter';

            % example 1
            run(fullfile('examples','examples_new','ex1_try_faster_run.m'))

            % example 2
            run(fullfile('examples','examples_new','ex2_adjust_cyl_obstacles.m'))

            % example 3
            run(fullfile('examples','examples_new','ex3_add_spheres_obstacles.m'))

            % example 4
            run(fullfile('examples','examples_new','ex4_add_walls.m'))

            % example 5
            run(fullfile('examples','examples_new','ex5_single_UAV.m'))

            % example 6
            run(fullfile('examples','examples_new','ex6_just_sphere_obstacles.m'))

            % example 7
            run(fullfile('examples','examples_new','ex7_active_goal.m'))

            % example 8
            run(fullfile('examples','examples_new','ex8_active_goal_with_dropouts.m'))

            % May want to add checks in the future.
        end
    end

end