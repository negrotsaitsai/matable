%% Create a ROS Master in MATLAB
clear
clc
rosinit

%% Start the simulator
sim = RobotSimulator('emptyMap');
pause(1)

%% Control parameters
xGoal = 20;
yGoal = 18;
goalRadius = 0.5;
K_angle = 3;
K_pos = 0.1;

%% Create subscribers
odomSub = rossubscriber('/odom');

%% Create publishers
velPub = rospublisher('/mobile_base/commands/velocity');

%% Initial calculation

% Receive latest odometry message
pose = getRobotPose(odomSub);

% Plot the initial goal position
hold(sim.Axes,'on');
goalLines = plot(sim.Axes, xGoal, yGoal, 'r*','MarkerSize',10);

% Find the initial distance to the goal
goalDist = sqrt((yGoal - pose(2))^2 + (xGoal - pose(1))^2);

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
    [v,w] = robotCtrl(pose,xGoal,yGoal,goalDist,K_pos,K_angle);
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
