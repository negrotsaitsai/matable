%% Create a ROS Master in MATLAB
close all; clear; clc; 
rosinit

%% Start the simulator
sim = RobotSimulator('simpleMap');

%% Create subscribers
% TODO %
% scanSub =

%% Create publishers
% TODO %
% velPub = 

%% Move the robot
velData = rosmessage(velPub);
velData.Angular.Z = 0.5;
send(velPub,velData);

%% Take another reading and display it with maximum range of 5 meters
% TODO %
% scanData = 

%% Close Simulator
close('Robot Simulator')

%% Shut down ROS
rosshutdown
