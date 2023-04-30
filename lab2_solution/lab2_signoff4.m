clear
clc

%% Setup robot
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model();
robot.writeTime(travelTime); % Write travel time

%% Program 

jointAngles = [0 0 0 0 ; 45 -15 -60 30 ; -45 0 15 -45 ; 105 -75 60 0];
fkArray = zeros(4, 4, 4);

for i = 1:height(jointAngles)
    disp(i)
    fkArray(:,:,i) = robot.getFK(jointAngles(i,:));
    robot.writeJoints(jointAngles(i,:)); % Write joint values
    disp(jointAngles(i,:))
    tic; % Start timer
    while toc < travelTime
        jointData = robot.getJointsReadings();
        q = jointData(1,:);
        model.plotArm(q, false);
        pause(0.25);
    end
    pause(10); % time to take photos
end

save('lab2_signoff4Data.mat', 'fkArray')
