function f = fitness_calc_copy(times_end, end_time, functional_end_time, f_min_pts)
% Functional copy of method _calculate_fitness in failure_modes_problem.py
% Can utilize outputs directly from
% wrapper_run_generation_exp_failure_modes.m, e.g.:
% f = fitness_calc_copy(out.times_end{1}, out.sim_config.end_time{1}, 
%                       functional_end_time, f_min_pts)
% The last two need to be pulled from nominal performance test results and
% user-specified min fitness.

total_points = 0;

for agent_end_time = times_end

    if agent_end_time > end_time
        points = 0;
    elseif agent_end_time <= functional_end_time
        points = 1;
    else
        points = 1 + (f_min_pts - 1) * (agent_end_time - functional_end_time) / ...
            (end_time - functional_end_time);
    end

    total_points = total_points + points;

end

f = total_points;

end