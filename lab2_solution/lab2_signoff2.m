clear
clc

%% Setup robot
robot = Robot(); % Creates robot object
robot.writeMotorState(true); % Write position mode
travelTime = 10;
robot.writeTime(travelTime)

%% Program 

robot.writeJoints([0 0 0 0])

tic
while(toc < travelTime)
    disp(robot.getCurrentFK());
end