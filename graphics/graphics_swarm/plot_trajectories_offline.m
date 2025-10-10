function [fig] = plot_trajectories_offline(pos_history, N, ...
                colors, fontsize, map, p_swarm)
% plot_trajectpries_offline - Plot trajectories of the agents.


% % For creating trajectory figures 1, 2 for paper (also see below)
% pos_history = pos_history(1:10:end,:);


x0 = 10; 
y0 = 10; 
width = 400;
height = 400;

fig = figure('Name','Offline swarm trajectories','NumberTitle','off');
hold on;
grid on;
box on;

% Plot environment; note that draw_obstacles also sets axes limits
if ~isempty(map) && ~isempty(p_swarm)
    fig = draw_obstacles(fig, map, p_swarm);
    if p_swarm.is_active_arena
        fig = draw_arena(fig, map);
    end
end

% Set axes limits and config (this probably needs improvement)
axis_len = map.width;
margin = axis_len/5;
axes_lim = [-margin, axis_len + margin, ...
    -margin, axis_len+margin, ...
    0, 1.2 * map.max_height];
axis equal;
axis(axes_lim);
view(2);

% Plot trajetcories
for agent = 1:N
    hold on;
    
    if ~isempty(colors)
        plot3(pos_history(:,(agent-1)*3+2), pos_history(:,(agent-1)*3+1), ...
            - pos_history(:,(agent-1)*3+3), 'Color', colors(:,agent));
        
        % % For creating trajectory figures 1, 2 for paper (also see 
        % % above)
        % plot3(pos_history(:,(agent-1)*3+2), pos_history(:,(agent-1)*3+1), ...
        %     - pos_history(:,(agent-1)*3+3), 'Color', colors(:,agent), ...
        %     'LineWidth', 2);

    else
        plot3(pos_history(:,(agent-1)*3+2), pos_history(:,(agent-1)*3+1), ...
            - pos_history(:,(agent-1)*3+3));
    end
    
end

xlabel('Y Position [m]','fontsize',fontsize);
ylabel('X Position [m]','fontsize',fontsize);
zlabel('Z Position [m]','fontsize',fontsize);
view(2);

set(fig,'units','pixels','position',[x0,y0,width,height]);
set(fig,'PaperPositionMode','auto');

end
