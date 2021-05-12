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
odomData = odomSub.LatestMessage;

% Unwrap position
position = odomData.Pose.Pose.Position;
x = position.X;
y = position.Y;

% Unwrap orientation
orientation = odomData.Pose.Pose.Orientation;
q = [orientation.W, orientation.X, ...
     orientation.Y, orientation.Z];
r = quat2eul(q);
theta = r(1); % Extract Z component

% Plot the initial goal position
hold(sim.Axes,'on');
goalLines = plot(sim.Axes, xGoal, yGoal, 'r*','MarkerSize',10);

% Find the initial distance to the goal
goalDist = sqrt((yGoal - y)^2 + (xGoal - x)^2);

%% Control loop
velData = rosmessage(velPub);
while (goalDist >= goalRadius)
    
    % Receive latest odometry message
    odomData = odomSub.LatestMessage;

    % Unwrap position
    position = odomData.Pose.Pose.Position;
    x = position.X;
    y = position.Y;

    % Unwrap orientation
    orientation = odomData.Pose.Pose.Orientation;
    q = [orientation.W, orientation.X, ...
         orientation.Y, orientation.Z];
    r = quat2eul(q);
    theta = r(1); % Extract Z component
    
    % Distance control
    goalDist = sqrt((yGoal - y)^2 + (xGoal - x)^2);
    velData.Linear.X = K_pos*goalDist;                  
            
    % Angle control
    thetaRef = atan2(yGoal - y, xGoal - x); %inverse tangent
    thetaError = angdiff(theta,thetaRef);
    velData.Angular.Z = K_angle*thetaError;
    
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
