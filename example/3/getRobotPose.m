function pose = getRobotPose(odomSub)

    % Get latest odometry message
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
    
    pose = [x y theta];

end