%% Configurations
clear
clc

travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object

% Define setpoint angles
eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45];
try
    jointAngles = [robot.getIK(eePoses(1,:)) ; robot.getIK(eePoses(2,:)); robot.getIK(eePoses(3,:)) ; robot.getIK(eePoses(1,:))];
catch
    error("End-Effector Pose Unreachable")
end

% Create Trajectory between setpoint angles
tj = TrajPlanner(jointAngles);
trajectories = tj.getCubicTraj(travelTime,5000);

%% Setup robot
pauseTime = height(eePoses)*travelTime/(height(trajectories)-1);
robot.writeMotorState(true); % Write position mode
robot.writeTime(pauseTime);

%% Program 

% Pre-allocate data
dataTime = zeros(2500, 1);
dataEePoses = zeros(2500, 4);
count = 1;

% Send to first vertex to start
robot.writeJoints(trajectories(1,2:end));
pause(5);
tic; % Start timer
% Go to each vertex in order
for i = 2:height(trajectories)
    robot.writeJoints(trajectories(i,2:end)); % Write joint values
    % Collect a reading periodically until the waypoint is reached
    while toc < (i-1) * pauseTime
        read = robot.getJointsReadings();
        q = read(1,:);
        dataTime(count) = toc;
        dataEePoses(count, :) = robot.getEEPos(q);
        count = count + 1;
    end
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataEePoses = dataEePoses(1:count-1,:);

% Save data to a file
data = struct("time", dataTime, "eePose", dataEePoses, "angles", trajectories);
save("lab3_signoff3_data.mat", "data");
