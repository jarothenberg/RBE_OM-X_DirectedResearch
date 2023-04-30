clear
clc

%% Setup robot
robot = Robot(); % Creates robot object
model = Model();
robot.writeMotorState(false); % Write position mode
%% Program 

% Do not move arm and just display joint angles

while true
    read = robot.getJointsReadings();
    q = read(1,:);
    model.plotArm(q, true);
    pause(0)
end