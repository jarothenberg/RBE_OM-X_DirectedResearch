%% Setup robot
robot = Robot(); % Creates robot object
robot.writeMotorState(false); % Write position mode
model = Model();
%% Program 

% Do not move arm and just display joint angles
while true
    q = robot.getJointsReadings();
    disp(q)
    model.plotArm(q, false);
    pause(0.5);
end