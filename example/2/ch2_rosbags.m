%% Accessing data from a ROS bag
clear; clc; close all;
bagFile = 'rosBagPose.bag';
bag = rosbag(bagFile);

%% Find list of topics recorded in the bag
bag.AvailableTopics

%% Select a subset of data from the bag
bagPose = select(bag,'Topic','/odom','Time',[70 90]);

%% Method1: Use readMessages method to extract data
poseMsgs = readMessages(bagPose);

%% Loop over cell array to extract position data from each message
x = zeros(1,numel(poseMsgs));
y = zeros(1,numel(poseMsgs));

for i = 1:numel(poseMsgs)
    
    odomMsg = poseMsgs{i};
    
    x(i) = odomMsg.Pose.Pose.Position.X;
    y(i) = odomMsg.Pose.Pose.Position.Y;
      
end

%% Plot position data
figure;
plot(x,y,'g*')
xlabel('X [meters]')
ylabel('Y [meters]')
title('Robot Position')

%% Method 2: Use timeseries to extract orientation data as timeseries
ts = timeseries(bagPose,...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X',...
    'Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z')

q = [ts.Data(:,1), ts.Data(:,2), ...
     ts.Data(:,3), ts.Data(:,4)];
% Convert quaternion to euler     
r = quat2eul(q);
theta = r(:,1);
% Extract time data
t = ts.Time;

%% Plot orientation data
figure;
plot(t,theta,'r-o')
xlabel('time [s]')
ylabel('Orientation angle [radians]')
title('Robot Orientation')

