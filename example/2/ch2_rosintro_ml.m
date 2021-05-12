%% Create a ROS Master in MATLAB
clear
clc
rosinit

%% Start the simulator
sim = RobotSimulator('emptyMap');

%% List all the eligible topics to subscribe from or publish to
rostopic list

%% Get more information about a topic including subscribers, publishers etc
rostopic info /odom

%% Subscribe to topic
odomSub = rossubscriber('/odom')

%% Get information again to see MATLAB as subscriber information added
rostopic info /odom

%% Receive data using 'receive'
odomData = receive(odomSub, 1);

%% Use the 'showDetails' method to show the contents of the message
showdetails(odomData)

%% Publish to topic
velPub = rospublisher('/mobile_base/commands/velocity')

%% Initialize the type and the content of data to publish
velData = rosmessage(velPub)
% velData.Angular.Z = 1;
velData.Linear.X = 10;

%% Send data
send(velPub, velData)

%% Receive data
odomData = receive(odomSub, 1);
% odomData = odomSub.LatestMessage;

%% Use the 'showDetails' method to show the contents of the message
showdetails(odomData)

%% List all the existing services
rosservice list

%% Calling a ROS Service
resetSim = rossvcclient('/sim/new_robot_pose')
call(resetSim)

%% Close Simulator
close('Robot Simulator')

%% Shut down ROS
rosshutdown
