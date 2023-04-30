clear
clc

%% Setup robot
robot = Robot(); % Creates robot object
travelTime = 3;
robot.writeTime(travelTime);

%% Program
qs = [0 0 0 0 ; 15 -45 -60 90 ; -90 15 45 -45];
for i=1:height(qs)
    q = qs(i,:);
    robot.writeJoints(q);
    pause(travelTime)
    disp(robot.getFK(q));
end