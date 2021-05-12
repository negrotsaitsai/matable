%% Create a ROS Master in MATLAB
clear
clc
rosinit

%% Start the simulator
sim = RobotSimulator('emptyMap');
pause(1)

%% Control parameters
xGoal = [18 20 10];
yGoal = [15 10 10];
goalRadius = 2;
K_angle = 3;
K_pos = 0.1;

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
goalDist = sqrt((yGoal(1) - y)^2 + (xGoal(1) - x)^2);

%% Control loop
velData = rosmessage(velPub);
k = 1;
done = 0;
while ~done
    
    % Receive latest odometry message
    pose = getRobotPose(odomSub);
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    % Control algorithm
    goalDist = sqrt((yGoal(k) - y)^2 + (xGoal(k) - x)^2);
    [v,w] = robotCtrl(pose,xGoal(k),yGoal(k),goalDist,K_pos,K_angle);
    velData.Linear.X = v;
    velData.Angular.Z = w;
    
    % Send control commands
    send(velPub,velData);
    
    % Plot results
    pathLines = plot(sim.Axes,x,y,'g*');
    axis equal
    pause(0.1);
    
    % TODO: Check whether to move to the next point or break out of the loop.
    % The loop terminates when "done" is set to 1.
              
end
disp('Goal reached within threshold!');

%% Cleanup
close('Robot Simulator')
rosshutdown
