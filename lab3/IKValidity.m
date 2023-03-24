% Validate IK by disabling motors and printing out FK/IK periodically
clear
clc

% Disable motors
rosshutdown
rosinit
robot = Robot();
robot.writeMotorState(false);

% Display IK/FK as arm is moved by hand every half second
while true
    jointAngles = robot.getJointsReadingsRadians();

    joints = robot.radsToDegs(jointAngles(1,:))
    fk = robot.getEEPos(jointAngles(1,:));
    try
        ik = robot.getIK(fk)
    catch
        error("End-Effector Pose Unreachable")
    end
    pause(0.5)
end