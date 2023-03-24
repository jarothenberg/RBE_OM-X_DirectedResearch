clear
clc

travelTime = 3;
robot = Robot();
robot.writeMotorState(true);
robot.writeTime(travelTime);
robot.writeJoints([-45 0 0 0]);
pause(travelTime);
robot.writeJoints(0);
pause(travelTime);

robot.writeTime(0);
pause(1);
robot.setOperatingMode("v");
robot.writeMotorState(true);
robot.writeVelocities([20 0 0 0]);
tic;
while toc < 3
    read = robot.getJointsReadings();
    read(1:2,:)
end
robot.writeVelocities(0);