classdef RangeSensor < robotics.algs.internal.GridAccess & handle
    %RangeSensor Simulation of range sensor readings
    %   This class simulates the readings of a range sensor, e.g. a LIDAR,
    %   in a given map. The opening angle of the range sensor is 180
    %   degrees and the number of returned readings within this opening
    %   angle can be adjusted dynamically.
    %   The simulated sensor also has a maximum range. If for a given
    %   sensor reading, no obstacle is found within the maximum range, that
    %   sensor reading will be returned as NaN.
    %
    %   RangeSensor properties:
    %       Map         - Map to be used by the range sensor
    %       MaxRange    - Maximum detection range of sensor (in meters)
    %       NumReadings - Number of sensor readings in the 180 opening cone
    %       SensorNoise - Gaussian sensor noise of range readings (stdev in meters)
    %       AngleSweep  - (Read-only) The sweep angles for the range sensor
    %
    %
    %   RangeSensor methods:
    %       getReading - Return the range readings
    %       showRays   - Show all simulated range beams
    %
    %   See also RobotSimulator.
    
    %   Copyright 2015 The MathWorks, Inc.
    
    properties
        %Map - Map to be used by the range sensor
        %   This is expected to be a robotics.BinaryOccupancyGrid object.
        Map
        
        %MaxRange - Maximum detection range of sensor (in meters)
        %   This also corresponds to the maximum length of each ray in world
        %   units. If an obstacle is encountered within this limit, the range
        %   is the euclidean distance to that obstacle. If it is beyond
        %   that limit, the range is NaN.
        %   Set this value to be appropriate to the desired characteristics of
        %   your sensor.
        %   Default: 15 meters
        MaxRange = 15
        
        %NumReadings - Number of sensor readings in the 180 opening cone
        %   Default: 11 readings
        NumReadings = 11
        
        %SensorNoise - Gaussian sensor noise of range readings (stdev in meters)
        %   Default: 0.05 meters
        SensorNoise = 0.05
    end
    
    properties (SetAccess = private)
        %AngleSweep - The sweep angles for the range sensor
        %   Here, pi/2 is straight ahead and 0 is to the right of the
        %   robot.
        AngleSweep
    end
    
    methods
        function obj = RangeSensor()
            %RangeSensor Constructor for range sensor object
            
            obj.AngleSweep = linspace(0, pi, obj.NumReadings);
        end
        
        function [ranges, angles, collisionLoc] = getReading(obj, robotPosition, orientationAngle)
            %getReading Return the range readings for simulated sensor
            %
            % Inputs:
            % robotPosition    - robot [x y] position in world coordinates
            % orientationAngle - world orientation of robot in radians.
            %
            % Outputs:
            % ranges - Euclidean distances from robot to obstacle, in world coordinates
            % angles - Absolute angles used for the sweep (i.e., not relative to robot orientation)
            %          Use obj.AngleSweep if you just want the relative angles
            % collisionLoc - Nx2 matrix of collision locations (world coordinates).
            
            angles = orientationAngle - pi/2 + obj.AngleSweep;
            ranges = zeros(1,length(angles)) * NaN;
            collisionLoc = zeros(length(angles),2);
            
            for i=1:length(angles)
                pos = robotPosition + obj.MaxRange*[cos(angles(i)) sin(angles(i))];
                [isObstacleFree,collisionPts] = internal.raycast(robotPosition, pos, ...
                    obj.Map.Grid, obj.Map.Resolution, obj.Map.GridLocationInWorld);
                
                if ~isObstacleFree
                    % The collision location is returned as center point of
                    % a grid cell
                    collisionLoc(i,:) = obj.Map.grid2world(collisionPts);
                    
                    % Project the center of the grid cell perpendicular to the ray
                    b = pos - robotPosition;
                    a = collisionLoc(i,:) - robotPosition;
                    a1 = dot(a,b) / dot(b,b) * b;
                    collisionLoc(i,:) = a1 + robotPosition;
                    
                    % Calculate actual ranges
                    ranges(i) = sqrt(sum(a.^2));
                end
            end
            
            % Add noise to ranges
            if obj.SensorNoise ~= 0
                ranges = obj.addGaussianNoise(ranges, 0, obj.SensorNoise);
            end
        end
        
        function showRays(obj, robotPosition, orientationAngle)
            %showRays - Show all simulated range beams
            figure; subplot(211); show(obj.Map, 'world'); hold on;
            plot(robotPosition(1), robotPosition(2), '^');
            
            [ranges, angles, collisionLoc] = obj.getReading(robotPosition, orientationAngle);
            for i=1:length(angles)
                if ~isnan(ranges(i))
                    plot([robotPosition(1) collisionLoc(i,1)], [robotPosition(2) collisionLoc(i,2)], ':');
                    plot(collisionLoc(i,1), collisionLoc(i,2), 'ro');
                end
            end
            
            subplot(212);
            plot(obj.AngleSweep, ranges, 'o-');
            
        end
    end
    
    methods        
        function set.Map(obj, map)
            %set.Map Setter function for Map property
            validateattributes(map, {'robotics.BinaryOccupancyGrid'}, ...
                {'nonempty', 'scalar'}, 'RangeSensor', 'Map');
            obj.Map = map;
        end
        function set.MaxRange(obj, maxRange)
            %set.MaxRange Setter function for MaxRange property            
            validateattributes(maxRange, {'double'}, ...
                {'nonempty', 'scalar'}, 'RangeSensor', 'MaxRange');
            obj.MaxRange = maxRange;
        end
        function set.NumReadings(obj, numReadings)
            %set.NumReadings Setter function for NumReadings property
            validateattributes(numReadings, {'double'}, ...
                {'nonempty', 'scalar'}, 'RangeSensor', 'NumReadings');            
            obj.NumReadings = numReadings;
            
            % Also update the angle sweep
            obj.AngleSweep = linspace(0, pi, obj.NumReadings); %#ok<MCSUP>
        end
        function set.SensorNoise(obj, sensorNoise)
            %set.SensorNoise Setter function for SensorNoise property
            validateattributes(sensorNoise, {'double'}, ...
                {'nonempty', 'scalar'}, 'RangeSensor', 'SensorNoise');  
            obj.SensorNoise = sensorNoise;
        end            
    end
    
    methods (Static, Access = private)
        function B = addGaussianNoise(A, mean, stdev)
            %addGaussianNoise Add Gaussian noise to input values
            %   B = addGaussianNoise(A, mean, stdev) adds Gaussian noise
            %   with a given MEAN and standard deviation STDEV to the input
            %   A and return the "noisy" output in B.
            
            B = A + randn(size(A))*stdev - stdev/2 + mean;   %# add gaussian noise
        end
    end
    
    
end

