%
% TestScript for Assignment 1
%

%% Define a small map
map = false(50);

% Add an obstacle
map (1:30, 20) = true;
% map(14,1:10)=true;
% map(18,1:16)=true;

start_coords = [2, 2];
dest_coords  = [18, 48];

%%
%close all;
%[route, numExpanded] = DijkstraGrid (map, start_coords, dest_coords);
% Uncomment following line to run Astar
[route, numExpanded] = AStarGrid (map, start_coords, dest_coords);

%HINT: With default start and destination coordinates defined above, numExpanded for Dijkstras should be 76, numExpanded for Astar should be 23.
