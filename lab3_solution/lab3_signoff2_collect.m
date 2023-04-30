%% Configurations
clear
clc

%% Setup robot
travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time

%% Program 

% Define setpoint angles
eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45];
try
    jointAngles = [robot.getIK(eePoses(1,:)) ; robot.getIK(eePoses(2,:)); robot.getIK(eePoses(3,:)) ; robot.getIK(eePoses(1,:))];
catch
    error("End-Effector Pose Unreachable")
end

% Pre-allocate data
dataTime = zeros(2500, 1);
dataAngles = zeros(2500, 4);
dataTransforms = zeros(4, 4, 2500);
dataEePoses = zeros(2500, 4);
count = 1;

% Send to first vertex to start
robot.writeJoints(jointAngles(1,:));
pause(travelTime);
tic; % Start timer
% Go to each vertex in order
for i = 2:4
    robot.writeJoints(jointAngles(i,:)); % Write joint values
    % Collect a reading periodically until the setpoint is reached
    while toc < (i-1) * travelTime
        read = robot.getJointsReadings();
        q = read(1,:);
        dataTime(count) = toc;
        dataAngles(count, :) = q;
        dataTransforms(:, :, count) = robot.getFK(q);
        dataEePoses(count, :) = robot.getEEPos(q);
        count = count + 1;
    end
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataAngles = dataAngles(1:count-1,:);
dataTransforms = dataTransforms(:, :, 1:count-1);
dataEePoses = dataEePoses(1:count-1,:);

% Save data to a file
data = struct('vertexAngles', jointAngles,'time', dataTime, 'angles', dataAngles, 'transforms', dataTransforms, 'eePos', dataEePoses);
save("lab3_signoff2_data.mat", "data");
