classdef RobotSimulator < handle
    %RobotSimulator Simulation for simple differential-drive robot
    %
    %   OBJ = RobotSimulator creates a simple simulator for a
    %   differential-drive robot. It receives velocity commands (messages
    %   of type 'geometry_msgs/Twist') on the '/mobile_base/commands/velocity' 
    %   topic, and sends odometry information (messages of type 'nav_msgs/Odometry') 
    %   to the '/odom' topic. In addition, it updates a figure window 
    %   showing the current position of the robot.
    %   The robot will be placed in the 'simpleMap' world.
    %
    %   OBJ = RobotSimulator(MAPNAME) loads a predefined map in the
    %   simulator. MAPNAME is a string and can be one of these: 'simpleMap'
    %   (default), 'emptyMap', 'borderMap', or 'complexMap'.
    %   The maps are assumed to have a resolution of 2 cells / meter.
    %
    %   OBJ = RobotSimulator(MAPNAME,MAPRESOLUTION) loads a predefined map
    %   with MAPNAME and sets the resolution to MAPRESOLUTION (in cells /
    %   meter). By default, the resolution is 2 cells / meter.
    %
    %   OBJ = RobotSimulator(BOG) loads the map defined by the
    %   robotics.BinaryOccupancyGrid object BOG into the simulator.
    %
    %   Notes:
    %   1) RobotSimulator expects MATLAB ROS functionality
    %      to be initialized. You can do this by calling ROSINIT (for a
    %      local ROS master) or invoking ROSINIT(MASTER_IP) to connect to an
    %      external master.
    %
    %   2) To stop the simulator, close the associated figure window or 
    %      delete the object.
    %
    %
    %   You can control the motion of the simulated robot and interface to 
    %   the simulator via ROS. The following topics and services are
    %   handled by the simulator:
    % 
    %   RobotSimulator Publications:
    %       /odom - Current robot odometry information (pose and velocity)
    %       /scan - The data from the simulated laser scanner
    %       /mobile_base/sensors/bumper - Indicates if the robot hits an obstacle
    %       
    %   RobotSimulator Subscriptions:
    %       /mobile_base/commands/velocity - Velocity commands to the robot
    %
    %   RobotSimulator Services:
    %       /sim/new_robot_pose - Move the robot into a new starting pose 
    %       /sim/reset_poses    - Reset the pose of the robot to the last known starting pose
    %
    %
    %   Some other behavior of the simulator can be changed by accessing
    %   various properties.
    %
    %   RobotSimulator properties:
    %       Map   - Map to be used by the robot
    %       Robot - Object representing a differential drive robot. Adjust
    %               the robot's maximum allowable accelerations and
    %               velocities.
    %       LaserSensor - Object representing the simulated range sensor.
    %                     Use it to adjust the characteristics of the
    %                     simulated range sensor.
    %       IsRangeDisplayed - If set to "true" will display range beams
    %
    %
    %   Usage:
    %     rosinit
    %     robotsim = RobotSimulator;
    %     delete(robotsim)
    %
    %   See also robotics.BinaryOccupancyGrid, RangeSensor, DifferentialDriveRobot.
    
    %   Copyright 2014-2015 The MathWorks, Inc.
        
    properties(Constant, Access = private)
        %Step - Integration step size (in seconds)
        %   This is also the rate of odometry message publishing
        Step = 0.05
      
        %PlotInterval - Interval between plot updates (in seconds)
        PlotInterval = 0.1
        
        %LineLengthDivider - Length of robot orientation line
        %   Length is determined by taking XLimits of graph divided 
        %   by LineLengthDivider.
        LineLengthDivider = 15
        
        % FigureName - Name of simulation figure window
        FigureName = 'Robot Simulator'
        
        % FigureTag - Tag used for the simulation figure window. 
        %   This is used to check if the window is already open.
        FigureTag = 'ROSRobotSimulator'
    end
    
    properties (SetAccess = private)
        %Map - Map to be used by the robot
        Map
        
        %Robot - Object representing a differential drive robot
        Robot
        
        %LaserSensor - Object representing the simulated range sensor
        LaserSensor
    end
    
    properties
        %IsRangeDisplayed - If set to "true" will display range beams
        %   Since the plot update for the range beams is quite expensive, 
        %   you can disable the display of the beams with this property.
        %   This is especially useful if you simulate a larger number of
        %   beams.
        IsRangeDisplayed = true
    end    
    
    properties (Hidden)
        %Axes - Handle to main plot axes
        Axes        
    end
    
    properties (Access = private)
        %InitialRobotState - Initial state of robot
        %   This is a 3-vector with elements [x y theta]
        InitialRobotState = [0 0 0]
        
        %KinematicsTimer - Timer for kinematics integration
        KinematicsTimer = timer.empty
        
        %PlotTimer - Timer triggering plot updates
        PlotTimer = timer.empty
        
        %VelCmdSubscriber - Subscriber for /mobile_base/commands/velocity messages
        VelCmdSubscriber
        
        %VelCmdPublisher - Publisher for /mobile_base/commands/velocity messages
        VelCmdPublisher
        
        %OdometryPublisher - Publisher for /odom topic        
        OdometryPublisher        
        
        %PoseMessage - nav_msgs/Odometry message for /odom topic
        PoseMessage

        % Properties related to laser scanner simulation
        
        %LaserScanPublisher - Publisher for /scan topic
        LaserScanPublisher
        
        %LaserScanMessage - Laser scan reading published on /scan
        %   Message type is sensor_msgs/LaserScan
        LaserScanMessage
        
        %BumperStatePublisher - Publisher for /mobile_base/sensors/bumper topic
        BumperStatePublisher
        
        %BumperStateMessage - std_msgs/Bool message indicating bumper / collision state
        BumperStateMessage
        
        %ResetSimulationService - Service server for /sim/reset_poses
        ResetSimulationService
        
        %RandomizeLocationService - Service server for /sim/new_robot_pose
        RandomizeLocationService
        
        %GazeboResetModelPosesService - Service server for /gazebo/reset_world
        GazeboResetModelPosesService
        
        %ScanRanges - The latest simulated range readings
        ScanRanges
        
        %ScanAngles - The latest scan angles (corresponding to ScanRanges)
        ScanAngles
        
        %ScanCollisionLoc - The latest 2D locations for laser hits
        %   These are the locations where the laser beams intersect
        %   obstacles in the map.
        ScanCollisionLoc        
        
        %Figure - Handle to figure window
        Figure        
        
        %PlotHandle1 - Handle to robot graphical representation
        PlotHandle1
        
        %PlotHandle2 - Handle to robot orientation graphical representation
        PlotHandle2
        
        %ScanLineHandles - Graphics handles for laser beam visualization
        ScanLineHandles = matlab.graphics.chart.primitive.Line.empty
        
        %ScanPointHandles - Graphics handles for laser end point visualization
        ScanPointHandles = matlab.graphics.chart.primitive.Line.empty
        
        %RandomLocationButton - Button for randomizing robot location
        RandomLocationButton
        
        %ResetSimulationButton - Button for resetting the robot simulation
        ResetSimulationButton
    end
    
    methods
        function obj = RobotSimulator(varargin)
            %RobotSimulator Constructor for simulator object
            
            existingFigures = findobj('type', 'figure', 'tag', RobotSimulator.FigureTag);
            if ~isempty(existingFigures)
                figure(existingFigures(1)); % bring figure to the front
                error('There is already an existing figure window for RobotSimulator.');
            end
            
            % Load all example maps from MAT file
            examplesFilePath = fullfile(fileparts(mfilename('fullpath')),'robotSimulatorExampleMaps.mat');
            exampleMaps = load(examplesFilePath);
            
            % Parse arguments to constructor
            narginchk(0,2);
                        
            switch nargin
                case 0
                    % RobotSimulator() syntax 
                    % Use default values
                    mapName = 'simpleMap';
                    mapResolution = 2;
                    obj.Map = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                case 1
                    % RobotSimulator(MAPNAME) or
                    % RobotSimulator(BOG) syntax
                    map = varargin{1};
                    if ischar(map)
                        % RobotSimulator(MAPNAME) syntax
                        mapResolution = 2;
                        mapName = validatestring(map, fieldnames(exampleMaps), 'RobotSimulator', 'mapName');
                        obj.Map = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                    else
                        % RobotSimulator(BOG) syntax
                        validateattributes(map, {'robotics.BinaryOccupancyGrid'}, {'nonempty','scalar'}, ...
                            'RobotSimulator', 'bog');
                        obj.Map = map;
                    end
                case 2
                    % RobotSimulator(MAPNAME, MAPRESOLUTION) syntax
                    mapName = varargin{1};
                    mapResolution = varargin{2};
                    mapName = validatestring(mapName, fieldnames(exampleMaps), 'RobotSimulator', 'mapName');
                    validateattributes(mapResolution, {'double'}, {'nonempty','scalar'}, 'RobotSimulator', 'mapResolution');
                    obj.Map = robotics.BinaryOccupancyGrid(exampleMaps.(mapName), mapResolution);
                otherwise
                    error('Invalid number of arguments to RobotSimulator.');
            end
                                                
            % Set initial state for robot
            obj.Robot = DifferentialDriveRobot;
            obj.Robot.setPose(obj.InitialRobotState);
                                         
            % Create ROS subscribers and publishers
            obj.VelCmdSubscriber = rossubscriber('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
            obj.VelCmdPublisher = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
            [obj.OdometryPublisher, obj.PoseMessage] = rospublisher('/odom', 'nav_msgs/Odometry');
            [obj.LaserScanPublisher, obj.LaserScanMessage] = rospublisher('/scan', 'sensor_msgs/LaserScan');
            [obj.BumperStatePublisher, obj.BumperStateMessage] = rospublisher('/mobile_base/sensors/bumper', 'std_msgs/Bool');
            
            % Create services for resetting the simulation
            % Both /sim/reset_poses and /gazebo/reset_world services will
            % do the same thing (reset the robot's start position and
            % velocity). The /gazebo/reset_world service is made available
            % to preserve parity with the simulated TurtleBot
            % representation in Gazebo.
            obj.ResetSimulationService = rossvcserver('/sim/reset_poses', 'std_srvs/Empty', ...
                @obj.resetSimulationServiceCallback);
            obj.GazeboResetModelPosesService = rossvcserver('/gazebo/reset_world', 'std_srvs/Empty', ...
                @obj.resetSimulationServiceCallback);
            
            % Create service for randomizing robot location
            obj.RandomizeLocationService = rossvcserver('/sim/new_robot_pose', 'std_srvs/Empty', ...
                @obj.randomizeLocationServiceCallback);
            
            % Create timers for execution loops
            obj.KinematicsTimer = timer('Period', obj.Step, ...
                'StartDelay', 1, ...
                'ExecutionMode', 'fixedrate', ...
                'TimerFcn', @obj.updateKinematics);
            
            obj.PlotTimer = timer('Period', obj.PlotInterval, ...
                'StartDelay', 1, ...
                'ExecutionMode', 'fixedrate' ,...
                'TimerFcn', @obj.updatePlot);
            
            setupLaserScanner(obj);            
            setupFigure(obj);
            
            % Start the two timing loops
            startTimers(obj);
            
            % Randomize location of robot and reset simulation
            randomizeLocation(obj);           
            resetSimulation(obj);
        end
        
        function delete(obj)
            %delete Called on object destruction or when the simulator window is closed
            %   Stop timers first (otherwise callbacks may be invoked with
            %   invalid Subscriber and Publisher).
            
            if ~isempty(obj.KinematicsTimer)
                stop(obj.KinematicsTimer);
                delete(obj.KinematicsTimer);
            end

            if ~isempty(obj.PlotTimer)
                stop(obj.PlotTimer);
                delete(obj.PlotTimer);
            end
            
            if ~isempty(obj.Figure)
                delete(obj.Figure);
            end
        end        
        
        function randomizeLocation(obj)
            %randomizeLocation Set the robot position to a random location
            %   This function ensures that the robot position will be in
            %   free space.

            isOccupied = true;
            iterations = 100;
            
            while isOccupied && iterations > 0
                x = obj.Map.XWorldLimits(1) + rand(1)*diff(obj.Map.XWorldLimits);
                y = obj.Map.YWorldLimits(1) + rand(1)*diff(obj.Map.YWorldLimits);
                theta = rand(1)*pi - pi/2;
                isOccupied = obj.Map.getOccupancy([x y]);
                iterations = iterations - 1;
            end
            
            % Update the initial robot state
            obj.InitialRobotState = [x y theta];
            obj.Robot.setPose(obj.InitialRobotState);
        end        
                
        function resetSimulation(obj)
            %resetSimulation Go back to initial conditions
            %   Set all velocity commands to zero as well.
            
            % Make sure that robot velocity is set to zero
            velmsg = rosmessage(obj.VelCmdPublisher);
            send(obj.VelCmdPublisher, velmsg);
            
            % Reset the initial robot state 
            obj.Robot.setPose(obj.InitialRobotState);
        end
    end
    
    %% Custom setters for class properties
    methods
        function set.IsRangeDisplayed(obj, rangeDisp)
            %set.IsRangeDisplayed Custom setter for IsRangeDisplayed property.
            validateattributes(rangeDisp,{'numeric', 'logical'}, ...
                    {'scalar','nonempty'}, 'RangeSensor', 'IsRangeDisplayed');
            obj.IsRangeDisplayed = logical(rangeDisp);
        end
    end
    
    %%
    methods (Access = private)
        function setupLaserScanner(obj)
            %setupLaserScanner Setup the simulated laser scanner
            
            obj.LaserSensor = RangeSensor();
            obj.LaserSensor.Map = obj.Map;
            senseAngles = obj.LaserSensor.AngleSweep;
            
            % Set values for laser scan message that are the same for each
            % simulation step
            msg = obj.LaserScanMessage;
            msg.Header.Stamp = rostime('now');
            msg.Header.FrameId = 'laser_scanner';
            msg.AngleMin = senseAngles(1)-pi/2;
            msg.AngleMax = senseAngles(end)-pi/2;
            msg.AngleIncrement = senseAngles(2) - senseAngles(1);
            msg.TimeIncrement = 0;
            msg.ScanTime = obj.Step;
            msg.RangeMin = 0;
            msg.RangeMax = obj.LaserSensor.MaxRange;           
        end
        
        function setupFigure(obj)    
            %setupFigure Setup figure window
            
            obj.Figure = figure('deletefcn', @obj.cleanup, 'Name', obj.FigureName, 'tag', obj.FigureTag);
            ax = axes('parent', obj.Figure);
            
            obj.Map.show('world', 'Parent', ax);
            hold(ax, 'on');
            title(ax, 'Circle = robot position, Line = robot orientation');
            obj.Axes = ax;
            
            % Create buttons in the GUI window
            obj.RandomLocationButton = uicontrol(obj.Figure, ...
                'Style', 'pushbutton', ...
                'String','Randomize Location', ...
                'Units', 'normalized', ...
                'Position', [0.80 0.01 0.2 0.05], ...
                'Callback', @obj.randomLocationButtonCallback);
            
            obj.ResetSimulationButton = uicontrol(obj.Figure, ...
                'Style', 'pushbutton', ...
                'String','Reset Simulation', ...
                'Units', 'normalized', ...
                'Position', [0.0 0.01 0.2 0.05], ...
                'Callback', @obj.resetSimulationButtonCallback);
            
            obj.PlotHandle1 = plot(obj.Axes, [0 1], [0 0], 'k');
            obj.PlotHandle2 = plot(obj.Axes, 0, 0, 'bo', 'MarkerFaceColor', [1 1 1]);
        end
        
        
        function startTimers(obj)
            %startTimers Launch the simulation loop
            
            start(obj.KinematicsTimer);
            start(obj.PlotTimer);
        end
        
        function updateKinematics(obj, ~, ~)
            %updateKinematicsUpdate the kinematic model of the robot

            controlInput = obj.VelCmdSubscriber.LatestMessage; % Twist message
            
            % Save last pose
            initialPose = obj.Robot.Pose;
            obj.Robot.updateKinematics(obj.Step, controlInput);
                       
            % Update robot state
            newPose = obj.Robot.Pose;
            
            % Check for collision with map
            try
                isOccupied = obj.Map.getOccupancy([newPose(1) newPose(2)]);
            catch
                % Cannot go outside the world
                isOccupied = true;
            end
            
            if isOccupied
                % Hit a obstacle. If we treat it as a "bounce" (i.e., move
                % the robot backwards), that causes noisy bumper state, so
                % don't update at all.
                obj.BumperStateMessage.Data = true;
                obj.Robot.setPose(initialPose);
            else
                obj.BumperStateMessage.Data = false;
            end
            
            send(obj.BumperStatePublisher, obj.BumperStateMessage);
            sendOdometryMessage(obj);
            
            % Update range reading
            pose = obj.Robot.Pose;
            [obj.ScanRanges, obj.ScanAngles, obj.ScanCollisionLoc] = obj.LaserSensor.getReading([pose(1) pose(2)], pose(3));
            obj.LaserScanMessage.Ranges = obj.ScanRanges;
            obj.LaserScanMessage.Header.Seq = obj.LaserScanMessage.Header.Seq + 1;
            send(obj.LaserScanPublisher, obj.LaserScanMessage);
        end
        
        
        function updatePlot(obj, ~, ~)
            %updatePlot Update the figure window with current robot
            %   Also plot the simulated range sensor rays.
            
            if ~obj.IsRangeDisplayed
                delete(obj.ScanLineHandles);
                obj.ScanLineHandles = matlab.graphics.chart.primitive.Line.empty;
                return;
            end
            
            x = obj.Robot.Pose(1);
            y = obj.Robot.Pose(2);
            theta = obj.Robot.Pose(3);
            
            % Find length orientation line
            ax = obj.PlotHandle1.Parent;
            lineLength = diff(ax.XLim) / obj.LineLengthDivider;
            xp = [x x+lineLength*cos(theta)];
            yp = [y y+lineLength*sin(theta)];
            
            set(obj.PlotHandle1, 'xdata', xp, 'ydata', yp);
            
            currentMarkerColor = obj.PlotHandle2.MarkerFaceColor;

            if obj.BumperStateMessage.Data 
                % Bumping into an obstacle. Alternate the color of the
                % marker between red and white
                if all(currentMarkerColor == [1 1 1])
                    markerFaceColor = [1 0 0];
                elseif all(currentMarkerColor == [1 0 0])
                    markerFaceColor = [1 1 1];
                end                                
            else
                markerFaceColor = [1 1 1];
            end
            set(obj.PlotHandle2, 'XData', x, 'YData', y, 'MarkerFaceColor', markerFaceColor);
            
            % Update range reading
            ranges = obj.ScanRanges;
            angles = obj.ScanAngles;
            collisionLoc = obj.ScanCollisionLoc;

            % Plot laser scan readings
            
            % If the number of range readings changes dynamically,
            % re-allocate the plot handles.
            if numel(obj.ScanLineHandles) ~= numel(ranges)
                delete(obj.ScanLineHandles);
                obj.ScanLineHandles = matlab.graphics.chart.primitive.Line.empty;
            end
            
            % If there are no existing plot handles, create them
            if isempty(obj.ScanLineHandles)
                for i = 1:length(angles)
                    obj.ScanLineHandles(end+1) = plot(obj.Axes, [x 0], [y 0], 'b:');
                end
            end
              
            for i = 1:length(angles)
                if isnan(ranges(i))
                    obj.ScanLineHandles(i).Visible = 'off';
                    continue;
                end
                
                obj.ScanLineHandles(i).XData = [x collisionLoc(i,1)];
                obj.ScanLineHandles(i).YData = [y collisionLoc(i,2)];
                obj.ScanLineHandles(i).Visible = 'on';
            end               
            drawnow('limitrate');
        end             
        
        function sendOdometryMessage(obj) 
            %sendOdometryMessage Publish odometry message based on robot pose
            
            x = obj.Robot.Pose(1);
            y = obj.Robot.Pose(2);
            theta = obj.Robot.Pose(3);
            
            % Get position handle and assign values
            position = obj.PoseMessage.Pose.Pose.Position;
            position.X = x;
            position.Y = y;
            
            % Convert Euler angles to quaternion
            q = eul2quat([theta 0 0]);
            
            % Get orientation handle and assign values
            orientation = obj.PoseMessage.Pose.Pose.Orientation;
            orientation.X = q(2);
            orientation.Y = q(3);
            orientation.Z = q(4);
            orientation.W = q(1);
            
            % Publish velocities as well
            twist = obj.PoseMessage.Twist.Twist;
            twist.Linear.X = obj.Robot.LinearVelocity;
            twist.Angular.Z = obj.Robot.AngularVelocity;
            
            % Publish the pose message
            header = obj.PoseMessage.Header;
            header.Seq = header.Seq + 1;
            send(obj.OdometryPublisher, obj.PoseMessage);
        end
        
        function response = resetSimulationServiceCallback(obj, ~, ~, response)
            %resetSimulationServiceCallback Executed when service call to /sim/reset_poses occurs
            
            % Reset the simulation
            obj.resetSimulation;
        end
        
        function response = randomizeLocationServiceCallback(obj, ~, ~, response)
            %randomizeLocationServiceCallback Executed when service call to /sim/new_robot_pose occurs
            
            % Randomize location of robot
            obj.randomizeLocation;
        end
    end
    
    %% All GUI-related methods
    methods (Access = private)
        function randomLocationButtonCallback(obj, ~, ~)
            %randomLocationButtonCallback Callback when user presses "Randomize location" button            
            randomizeLocation(obj);
        end
        
        function resetSimulationButtonCallback(obj, ~, ~)
            %resetSimulationButtonCallback Callback when user presses "Reset simulation" button            
            resetSimulation(obj);
        end
        
        function cleanup(obj, ~, ~)
            %cleanup Figure window was closed, so delete this object
            delete(obj);
        end        
    end    
    
end
