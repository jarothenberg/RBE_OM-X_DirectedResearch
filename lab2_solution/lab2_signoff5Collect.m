clear
clc

%% Setup robot
travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time

%% Program 

jointAngles = [0 -45.0879 60.7324 51.9431 ; 0 9.82285 52.2070 -44.0332 ; 0 10.2832 -3.6035 -78.1348; 0 -45.0879 60.7324 51.9431];

data = zeros(100000, 8);
count = 1;   
robot.writeJoints(jointAngles(1,:));
pause(travelTime);

tic; % Start timer
for i = 2:4
    robot.writeJoints(jointAngles(i,:)); % Write joint values
    while toc < (i-1) * travelTime
        read = robot.getJointsReadings();
        q = read(1,:);
        eePose = robot.getEEPos(q); % Extract translation vector
        d = eePose(1:3);
        data(count, :) = [toc q d];
        count = count + 1;
    end
end
data = data(1:count-1,:);
saveData = struct('data', data, 'qs', jointAngles);
save("lab2_signoff5Data.mat", "saveData");