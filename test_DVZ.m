clear;
clc;

%% Properties

% obstacle
Obstacles = struct('x', [], 'y', []);

Map.res = 0.1;
Map.limit = struct( ...
    'x', struct('min', -10, 'max', 10), ...
    'y', struct('min', -10, 'max', 10)  ...
    );
Map.map = zeros( ...
    round((Map.limit.y.max-Map.limit.y.min)/Map.res) + 1, ...
    round((Map.limit.x.max-Map.limit.x.min)/Map.res) + 1  ...
    );

% Robot properties
Robot.length = 0.78;
Robot.width  = 0.51;
Robot.position.x = 0;
Robot.position.y = 0;
Robot.orientation = 0;
[Robot.body.x, Robot.body.y] = rectangleObstacle( ...
    Robot.position.x, Robot.position.y, ...
    Robot.width, Robot.length, Robot.orientation);
Robot.velocity = struct('linear', 0, 'angular', 0); 
Robot.velocityLimit = struct('linear', 0.7, 'angular', 0.5);

numTheta = 1000;
theta = linspace(-pi+2*pi/numTheta, pi, numTheta);

% static virtual zone (SVZ)
SVZ.x = zeros(1, numTheta);
SVZ.y = zeros(1, numTheta);
SVZ.a = 0;      % semi-major axis
SVZ.b = 0;      % semi-minor axis
SVZ.ka = Robot.length * 10;     % gain for a
SVZ.kb = Robot.width * 10;      % gain for b
SVZ.dist = zeros(1, numTheta);

% deformable virtual zone (DVZ)
DVZ.x = zeros(1, numTheta);
DVZ.y = zeros(1, numTheta);
DVZ.a = 0;      % semi-major axis
DVZ.b = 0;      % semi-minor axis
DVZ.ka = Robot.length * 10;
DVZ.kb = Robot.width * 10;
DVZ.dist = zeros(1, numTheta);

%% Set obstacles

% Set up the static obstacles
[Obstacles(1).x, Obstacles(1).y] = rectangleObstacle(0, 2, 1, 2, 0);
[Obstacles(2).x, Obstacles(2).y] = rectangleObstacle(3, -2, 1, 2, 0);
% [Obstacles(3).x, Obstacles(3).y] = rectangleObstacle(-5, -5, 1, 2, 0);
% [Obstacles(4).x, Obstacles(4).y] = rectangleObstacle(5, -5, 1, 2, 0);

% Initialize the map
for idx = 1:length(Obstacles)
    [i_UR, j_UR] = coord2mat(Obstacles(idx).x(3), Obstacles(idx).y(3), ...
        Map.limit.x.max, Map.limit.y.max, Map.res);
    [i_UL, j_UL] = coord2mat(Obstacles(idx).x(4), Obstacles(idx).y(4), ...
        Map.limit.x.max, Map.limit.y.max, Map.res);
    [i_BL, j_BL] = coord2mat(Obstacles(idx).x(5), Obstacles(idx).y(5), ...
        Map.limit.x.max, Map.limit.y.max, Map.res);
    Map.map(i_UL:i_BL, j_UL:j_UR) = ones(length(i_UL:i_BL), length(j_UL:j_UR));
end

%% Obtain positions of the obstacles around the robot

SVZ.a = 7.8;
SVZ.b = 5.1;
SVZ.dist = ((SVZ.a * cos(theta)).^2 + (SVZ.b * sin(theta)).^2).^0.5;
SVZ.x = Robot.position.x ...
    + SVZ.a * cos(Robot.orientation) * cos(theta) ...
    - SVZ.b * sin(Robot.orientation) * sin(theta);
SVZ.y = Robot.position.y ...
    + SVZ.a * sin(Robot.orientation) * cos(theta) ...
    + SVZ.b * cos(Robot.orientation) * sin(theta);

DVZ.x = SVZ.x;
DVZ.y = SVZ.y;
DVZ.dist = SVZ.dist;

numBeamPoint = numTheta / 10;
beam.x = zeros(numTheta, numBeamPoint);
beam.x(:, numBeamPoint) = (SVZ.dist .* cos(theta+Robot.orientation)).';
beam.x(:, 2:numBeamPoint-1) = ((SVZ.dist/(numBeamPoint-1)) ...
    .* cos(theta+Robot.orientation)).' .* (1:numBeamPoint-2);
beam.x = beam.x + Robot.position.x;

beam.y = zeros(numTheta, numBeamPoint) + Robot.position.y;
beam.y(:, numBeamPoint) = (SVZ.dist .* sin(theta+Robot.orientation)).';
beam.y(:, 2:numBeamPoint-1) = ((SVZ.dist/(numBeamPoint-1)) ...
    .* sin(theta+Robot.orientation)).' .* (1:numBeamPoint-2);
beam.y = beam.y + Robot.position.y;

[beam.i, beam.j] = coord2mat(beam.x, beam.y, Map.limit.x.max, Map.limit.y.max, Map.res);
mapIndices = sub2ind(size(Map.map), beam.i, beam.j);
beam.point = Map.map(mapIndices);
for idx = 1:numTheta
    obstacleIndices = find(beam.point(idx,:) == 1);
    if ~isempty(obstacleIndices)
        obs.index = mapIndices(idx, obstacleIndices(1));
        [obs.i, obs.j] = ind2sub(size(Map.map), obs.index);
        [obs.x, obs.y] = mat2coord(obs.i, obs.j, ...
            Map.limit.x.max, Map.limit.y.max, Map.res);
        DVZ.x(idx) = obs.x;
        DVZ.y(idx) = obs.y;
        DVZ.dist(idx) = ((obs.x - Robot.position.x)^2 ...
            + (obs.y - Robot.position.y)^2)^0.5;
    end
end

%% Plotting

axis([-10 10 -10 10]); hold on; grid on;

% Plot the body of the mobile robot
plot(Robot.body.x, Robot.body.y);

% Plot the DVZ field
plot(SVZ.x, SVZ.y, ':', 'Color', '#00FF00');
plot(DVZ.x, DVZ.y, '-', 'Color', '#77AC30');

% Plot the heading orientation
plot([Robot.position.x, Robot.position.x+cos(Robot.orientation)], ...
     [Robot.position.y, Robot.position.y+sin(Robot.orientation)], ...
    "Color", 'r' ...
    );

% Plot the obstacles
for i = 1:length(Obstacles)
    plot(Obstacles(i).x, Obstacles(i).y, 'Color', 'k');
end


%% Functions
function [i,j] = coord2mat(x, y, xMax, yMax, res)
    i = round((yMax-y)/res)+1;
    j = round((xMax+x)/res)+1;
end

function [x,y] = mat2coord(i, j, xMax, yMax, res)
    x =  (j-1)*res - xMax;
    y = -(i-1)*res + yMax;
end

function [x, y] = rectangleObstacle(centerX, centerY, height, width, theta)
    % Check if the input values are valid
    if height <= 0 || width <= 0
        error('Height and width must be positive values.');
    end

    % Assume the bottom-left corner of the rectangle is at the origin (0,0)
    originalX = centerX + [-width/2, width/2, width/2, -width/2, -width/2];
    originalY = centerY + [-height/2, -height/2, height/2, height/2, -height/2];
    x = cos(theta) * originalX - sin(theta) * originalY;
    y = sin(theta) * originalX + cos(theta) * originalY;
end