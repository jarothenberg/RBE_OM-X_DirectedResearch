clear
clc

robot = Robot()
robot.writeMotorState(true);
robot.writeTime(0);
robot.writeJoints([0 0 0 0]);
pause(1);

robot.setOperatingMode("v");
robot.writeVelocities([20 0 0 0]);
tic;
while toc < 5
    read = robot.getJointsReadings();
    read(1:2,:)
end