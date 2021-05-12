classdef DifferentialDriveRobot < handle
    %DifferentialDriveRobot Simulation of kinematic and dynamic behavior of differential-drive robot
    %   This class handles the pose update of a differential-drive robot
    %   based on desired linear and angular velocities
    %   It also defines some maximum allowable accelerations and velocities
    %   that are representative of the TurtleBot 2 robot.
    %
    %   DifferentialDriveRobot properties:
    %       Pose                - (Read-only) Current pose of robot [x, y, theta]
    %       LinearVelocity      - (Read-only) Current linear forward velocity of robot (in m/s)
    %       AngularVelocity     - (Read-only) Current angular velocity of robot (in rad/s)
    %       LinearAcceleration  - (Read-only) Current linear forward acceleration (in m/s^2)
    %       AngularAcceleration - (Read-only) Current angular acceleration (in rad/s^2)
    %
    %       MaxLinearVelocity      - Maximum allowable linear velocity for robot (in m/s)
    %       MaxAngularVelocity     - Maximum allowable angular velocity for robot (in rad/s)
    %       MaxLinearAcceleration  - Maximum linear acceleration (in m/s^2)
    %       MaxAngularAcceleration - Maximum angular acceleration (in rad/s^2)
    %
    %
    %   DifferentialDriveRobot methods:
    %       setPose          - Set the pose of the robot
    %       updateKinematics - Propagate the kinematic model of the robot
    %
    %   See also RobotSimulator.    
    
    %   Copyright 2015 The MathWorks, Inc.
    
    properties (SetAccess = private)
        %Pose - Current pose of robot [x, y, theta]
        Pose = [0 0 0]
        
        %LinearVelocity - Current linear forward velocity of robot (in m/s)
        LinearVelocity = 0
        
        %AngularVelocity - Current angular velocity of robot (in rad/s)
        AngularVelocity = 0
        
        %LinearAcceleration - Current linear forward acceleration (in m/s^2)
        LinearAcceleration = 0
        
        %AngularAcceleration - Current angular acceleration (in rad/s^2)
        AngularAcceleration = 0
    end
    
    properties
        %MaxLinearVelocity - Maximum allowable linear velocity for robot (in m/s)
        %   The default of 0.65 m/s is based on specs for the TurtleBot 2.
        %   Default: 0.65 m/s
        MaxLinearVelocity = 0.65
        
        %MaxAngularVelocity - Maximum allowable angular velocity for robot (in rad/s)
        %   The default of 3.1415 rad/s is based on specs for the TurtleBot 2.
        %   Default: 3.1415 rad/s
        MaxAngularVelocity = pi
        
        %MaxLinearAcceleration - Maximum linear acceleration (in m/s^2)
        %   Default: 1.0 m/s^2
        MaxLinearAcceleration = 1.0
        
        %MaxAngularAcceleration - Maximum angular acceleration (in rad/s^2)
        %   Default: 1.0 rad/s^2
        MaxAngularAcceleration = 1.0
    end
    
    properties (Access = private)
        %LastVelocityCmd - Last valid velocity command that was received
        LastVelocityCmd = rosmessage('geometry_msgs/Twist')
        
        %TimeSinceLastCmd - Elapsed time since the last velocity command
        %   This counts the time (in second), since the last unique
        %   velocity command was received.
        TimeSinceLastCmd = 0
        
        %MaxCommandTime - The maximum time interval between velocity commands
        %   If more time (in seconds) than MaxCommandTime elapses between
        %   unique velocity commands, the robot will be stopped.
        %   Default: 1 second
        MaxCommandTime = 1
    end
    
    methods
        function setPose(obj, pose)
            %setPose Set the pose of the robot
            %   This will also reset the linear and angular velocity to
            %   zero.
            
            validateattributes(pose, {'double'}, {'nonempty', 'vector', 'numel', 3}, 'setRobotPose', 'pose');
            
            obj.Pose = pose;
            obj.LinearVelocity = 0;
            obj.AngularVelocity = 0;
        end
        
        function updateKinematics(obj, dt, velCmd)
            %updateKinematics Propagate the kinematic model of the robot
            
            if isempty(velCmd)
                % No valid velocity command. Set commanded velocities to 0.
                v = 0;
                w = 0;
            else
                % Set desired velocities based on control input
                v = velCmd.Linear.X;
                w = velCmd.Angular.Z;
                
                if obj.LastVelocityCmd.isImplIdenticalTo(velCmd)
                    % No new velocity command. Increment wait time.
                    obj.TimeSinceLastCmd = obj.TimeSinceLastCmd + dt;
                else
                    % This is a new velocity command.
                    obj.TimeSinceLastCmd = 0;
                    obj.LastVelocityCmd = velCmd;
                end
            end
            
            % If no velocity command is received within some time, stop the robot.
            if obj.TimeSinceLastCmd > obj.MaxCommandTime
                v = 0;
                w = 0;
            end
            
            % v and w are the desired velocities (control inputs)
            % Determine if the set point is bigger or smaller than the
            % current velocity and adjust the robot's acceleration
            % accordingly.
            % For illustration, consider the following scenarios:
            % v = 2, LinearVelocity = 1.5, linVelDiff = 0.5    --> accelerate forward
            % v = 2, LinearVelocity = -1.5, linVelDiff = 3.5   --> accelerate forward
            % v = -2, LinearVelocity = 1.5, linVelDiff = -3.5  --> accelerate backwards
            % v = -2, LinearVelocity = -1.5, linVelDiff = -0.5 --> accelerate backwards
            linVelDiff = v - obj.LinearVelocity;
            angVelDiff = w - obj.AngularVelocity;
            
            % Determine acceleration that should be applied to robot.
            % Limit it by the maximum allowable acceleration.
            linAcc = sign(linVelDiff) * min(10.0 * abs(linVelDiff), obj.MaxLinearAcceleration);
            angAcc = sign(angVelDiff) * min(10.0 * abs(angVelDiff), obj.MaxAngularAcceleration);
            obj.LinearAcceleration = linAcc;
            obj.AngularAcceleration = angAcc;
            
            % Based on the commanded accelerations, calculate new velocities.
            linVel = obj.LinearVelocity + linAcc * dt;
            angVel = obj.AngularVelocity + angAcc * dt;
            
            % Limit velocities to maximum allowable values
            obj.LinearVelocity = sign(linVel) * min(abs(linVel), obj.MaxLinearVelocity);
            obj.AngularVelocity = sign(angVel) * min(abs(angVel), obj.MaxAngularVelocity);
            
            % Set velocities to zero if they are within a threshhold
            if abs(obj.LinearVelocity) < 1e-5
                obj.LinearVelocity = 0;
            end
            if abs(obj.AngularVelocity) < 1e-5
                obj.AngularVelocity = 0;
            end
            
            % Propagate robot state change based on velocities
            dx = dt * obj.LinearVelocity*cos(obj.Pose(3));
            dy = dt * obj.LinearVelocity*sin(obj.Pose(3));
            dtheta = dt * obj.AngularVelocity;
            
            % Update robot state accordingly
            obj.Pose(1) = obj.Pose(1) + dx;
            obj.Pose(2) = obj.Pose(2) + dy;
            obj.Pose(3) = robotics.internal.wrapToPi(obj.Pose(3) + dtheta);
        end
    end
    
end

