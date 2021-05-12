%% Load sample map data
clc
clear
load robotSimulatorExampleMaps

%% Create and show an occupancy grid from a logical matrix
resolution = 1;
map = robotics.BinaryOccupancyGrid(simpleMap,resolution);
show(map);

%% Inflate the occupancy grid to account for the robot radius
robotRadius = 0.1;
inflate(map,robotRadius);
show(map);

%% Create and show a probabilistic roadmap
planner = robotics.PRM(map);
planner.NumNodes = 100;      
planner.ConnectionDistance = 10;
show(planner);

%% Find and show a path between start and goal points
startPoint = [5 5];
goalPoint = [20 9];
path = findpath(planner,startPoint,goalPoint)
show(planner);