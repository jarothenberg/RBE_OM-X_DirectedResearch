clc
clear

travelTime = 5;
acc_time = 2;

robot = Robot();
robot.toggleTorque(false);
robot.setOperatingMode('p');
robot.toggleTorque(true);
robot.writeJoints([90 0 0 0]);
disp("hi1")
pause(1)
robot.writeTime(travelTime, 2.5);
disp("hi2")
tic
robot.writeJoints([-90 0 0 0]);
read = robot.getJointsReadings();
motor1Pos = read(1,1);
while motor1Pos > -89.965
    read = robot.getJointsReadings();
    motor1Pos = read(1,1);
end
disp(toc)
disp("hi3")
pause(travelTime)
robot.toggleTorque(false);