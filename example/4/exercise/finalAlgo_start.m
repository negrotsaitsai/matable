%% Create a ROS Master in MATLAB
clc; clear;
rosinit

%% Start the simulator
resolution = 1;
sim = RobotSimulator('simpleMap',resolution);
pause(1)

%% Control parameters
xGoal = 15;
yGoal = 5;
goalRadius = 0.1;

%% Create subscribers
odomSub = rossubscriber('/odom');

%% Create publishers
velPub = rospublisher('/mobile_base/commands/velocity');

%% TODO: Create an occupancy grid from the "simpleMap" variable, 
% using the "resolution" variable.
load robotSimulatorExampleMaps
% map = 

%% TODO: Inflate the occupancy grid to account for the robot radius
robotRadius = 0.1;

%% TODO: Create a probabilistic roadmap with 100 nodes and a maximum
% connection distance of 10
% planner =

%% Find a path to the goal point

% Get initial robot pose
pose = getRobotPose(odomSub);
x = pose(1);
y = pose(2);
theta = pose(3);

% Find path
startPoint = [x y];
goalPoint = [xGoal yGoal];
%%%%%%%%%%%%%%%%%%%%
% TODO: path = ??? %
%%%%%%%%%%%%%%%%%%%%

% Plot the path
hold(sim.Axes,'on');
plot(sim.Axes, path(:,1), path(:,2), 'r*','MarkerSize',10);

% Find initial distance to the goal
goalDist = sqrt((yGoal - y)^2 + (xGoal - x)^2);

%% TODO: Create pure pursuit controller
% DesiredLinearVelocity = 0.5; MaxAngularVelocity = 1; LookaheadDistance = 1;
% controller =

%% Control loop
velData = rosmessage(velPub);
while (goalDist >= goalRadius)
    
    % Receive latest odometry message
    pose = getRobotPose(odomSub);
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    % Control algorithm
    goalDist = sqrt((yGoal - y)^2 + (xGoal - x)^2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % TODO: Use the pure pursuit controller
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    velData.Linear.X = v;
    velData.Angular.Z = w;
    
    % Send control commands
    send(velPub,velData);
    
    % Plot results
    plot(sim.Axes,x,y,'g*');
    axis equal
    pause(0.1);
           
end
disp('Goal reached within threshold!');

%% Cleanup
close('Robot Simulator')
rosshutdown
release(controller)
