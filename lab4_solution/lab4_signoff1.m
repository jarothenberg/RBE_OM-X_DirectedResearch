clear
clc

robot = Robot(); % Creates robot object

% Angles which would cause the robot to reach overhead singularity
overheadPos = [0 -atan2d(24,128) atan2d(24,128)-90 0];
% Angles which would cause the robot to reach stretched singularity
stretchedPos = [0 90-atan2d(24,128) atan2d(24,128)-90 0];

% Home position
homePos = [0 0 0 0];

eePoses = [overheadPos ; stretchedPos ; homePos];

for i=1:height(eePoses)
    % Calculates Jacobian at overhead pose
    J = robot.getJacobian(eePoses(i,:))
    % Calculates determinant of first 3x3 of jacobian
    JforDet = J(1:3,1:3);
    % Calculates determinant of that Jacobian
    JDet = det(JforDet)
end
