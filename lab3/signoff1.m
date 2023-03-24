% Code for signoff 1 to print the calculated FK/IK for known positions.

clear
clc
rosshutdown
rosinit
robot = Robot();

% 0 0 0 0
%%{
xe = 274;
ye = 0;
ze = 204.8;
phi = 0;
%}

try
    ik = robot.getIK([xe ye ze phi])
catch
    error("End-Effector Pose Unreachable")
end
fk = robot.getFK(robot.degsToRads(ik))

% 45 -15 -60 30
%%{
xe = 78.7032;
ye = 78.7032;
ze = 415.4938;
phi = -(-15 + (-60) + 30); % 45
%}

try
    ik = robot.getIK([xe ye ze phi])
catch
    error("End-Effector Pose Unreachable")
end
fk = robot.getFK(robot.degsToRads(ik))

% -45 0 15 -45
%%{  
xe = 178.8231;
ye = -178.8231;
ze = 235.6718;
phi = -(0 + 15 + (-45));
%}

try
    ik = robot.getIK([xe ye ze phi])
catch
    error("End-Effector Pose Unreachable")
end
fk = robot.getFK(robot.degsToRads(ik))