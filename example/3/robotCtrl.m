function [v,w] = robotCtrl(pose,xGoal,yGoal,goalDist,K_pos,K_angle)

    % Extract x, y, and theta    
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    
    % Distance control
    v = K_pos*goalDist;                  

    % Angle control
    thetaRef = atan2(yGoal - y, xGoal - x);
    thetaError = angdiff(theta,thetaRef);
    w = K_angle*thetaError;

end