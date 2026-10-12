% Create 2D video that animates the trajectories in a very basic way
% (use x-y plane)

clear, close all

% .mat files with the data we want (p_swarm, pos_history, min_d_obs)
mydata = {
    'results\results_swarm\ex5_single_UAV_2023_05_03_16_55_34\performance.mat'  % Specify performance.mat file path
    };

movienames = {
    'original';
    'modified'
    };

% Iterate through our data and create videos
for ii = 1:length(mydata)

    generateTrajAnimation(mydata{ii}, movienames{ii});

end

function generateTrajAnimation(fileStr, vidTitle)

% Load data
load(fullfile(fileStr), 'pos_history', 'p_swarm', 'min_d_obs')

% Create baseline figure (hard-coding limits - not ideal)
ylimits = [-50, 350];
xlimits = [-75, 375];
figure
tempFigureSize = get(gcf,'Position');
sizeMult = 1.5;
set(gcf, 'Position', [tempFigureSize(1), tempFigureSize(2), ...
    tempFigureSize(3)*sizeMult, tempFigureSize(4)*sizeMult]);
xlim(xlimits)
ylim(ylimits)
grid on

hold on

% Plot obstacles
if ~isempty(p_swarm.cylinders)
    for ii = 1:size(p_swarm.cylinders,2)
        drawCircle(p_swarm.cylinders(2,ii), p_swarm.cylinders(1,ii), p_swarm.cylinders(3,ii));
    end
end

% Initialize video
v = VideoWriter(vidTitle, 'MPEG-4');
v.Quality = 100;
open(v)

framesPerSec = 30;
secDuration = 7;
numFrames = framesPerSec * secDuration;
stepSize = floor(size(pos_history,1)/numFrames);

% Iterate through frames in the video; plot positions of agents
nb = p_swarm.nb_agents;
for jj = 1:stepSize:size(pos_history,1)

    % Get x and y coordinates of agents at this time step
    xCoord = pos_history(jj, 2:3:(2+(3*(nb-1))));
    yCoord = pos_history(jj, 1:3:(1+(3*(nb-1))));

    % plot position in NED; check if collision
    for kk = 1:nb

        % Check if distance to obstacle is less than collision distance
        if min_d_obs(jj,kk) <= p_swarm.r_coll
            plot(xCoord(kk), yCoord(kk), 'r.', 'MarkerSize', 10)
        else
            plot(xCoord(kk), yCoord(kk), 'b.', 'MarkerSize', 10)
        end

    end

    %         plot(pos_history(jj, 2:3:(2+(3*(nb-1)))), ...
    %             pos_history(jj, 1:3:(1+(3*(nb-1)))), 'b.')

    % Draw moving icons
    agentIcons = plot(xCoord, yCoord, 'mo', 'MarkerSize', 10, 'LineWidth', 2);

    % save frame to animation
    drawnow
    frame = getframe(gcf);
    writeVideo(v, frame);

    % Delete moving icons
    delete(agentIcons)

end

% Close video
close(v)

end

function drawCircle(x,y,r)

ang=0:0.01:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp, 'k-');

end