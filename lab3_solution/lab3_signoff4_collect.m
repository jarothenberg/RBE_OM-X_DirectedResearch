%% Configurations
clear
clc

travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object

% Define setpoint poses
eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45 ; 25 -100 150 -60];

% Create trajectory of poses
tj = TrajPlanner(eePoses);
trajectories = tj.getCubicTraj(travelTime,5000);

% Convert task space trajectory to a joint space one to follow
trajectoriesAngles = trajectories;
for i = 1:height(trajectoriesAngles)
    try
        trajectoriesAngles(i,2:end) = robot.getIK(trajectories(i,2:end));
    catch
        error("End-Effector Pose Unreachable")
    end
end

%% Setup robot
pauseTime = (height(eePoses)-1)*travelTime/(height(trajectoriesAngles)-1);
robot.writeMotorState(true); % Write position mode
robot.writeTime(pauseTime);

%% Program 
% Pre-allocate data
dataTime = zeros(2500, 1);
dataEePoses = zeros(2500, 4);
count = 1;
% Send to first vertex to start
robot.writeJoints(trajectoriesAngles(1,2:end));
pause(5);
tic; % Start timer
% Go to each vertex in order
for i = 2:height(trajectoriesAngles)
    robot.writeJoints(trajectoriesAngles(i,2:end)); % Write joint values
    % Collect a reading periodically until the setpoint is reached
    while toc < (i-1) * pauseTime
        %disp(toc)
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
data = struct("time", dataTime, "eePose", dataEePoses, "angles", trajectoriesAngles);
save("lab3_signoff4_data.mat", "data");