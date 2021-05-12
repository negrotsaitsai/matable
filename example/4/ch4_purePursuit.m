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

%% Create subscribers
odomSub = rossubscriber('/odom');

%% Create publishers
velPub = rospublisher('/mobile_base/commands/velocity');

%% Initial calculation
% Receive latest odometry message
pose = getRobotPose(odomSub);
x = pose(1);
y = pose(2);
theta = pose(3);  

% Plot the initial goal position
hold(sim.Axes,'on');
plot(sim.Axes, xGoal, yGoal, 'r*','MarkerSize',10);

% Find the initial distance to the goal
goalDist = sqrt((yGoal - y)^2 + (xGoal - x)^2);

%% Create pure pursuit controller
controller = robotics.PurePursuit;
controller.Waypoints = [x y;xGoal yGoal];
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 3;

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
    [v,w] = controller([x,y,theta]); 
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
