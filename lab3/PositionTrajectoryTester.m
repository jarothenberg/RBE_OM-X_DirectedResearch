%% Configurations
clear
clc

numPoints = 500;
travelTime = 3; % Defines the travel time
robot = Robot(); % Creates robot object

% Define setpoint poses
eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45 ; 25 -100 150 -60];

% Create trajectory of poses
tj = TrajPlanner(eePoses);
trajectories = tj.getQuinticTraj(travelTime,numPoints);

% Convert task space trajectory into a joint space one to follow
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
dataJoints = zeros(2, 4, 2500);
count = 1;
% Send to first vertex to start
robot.writeJoints(trajectoriesAngles(1,2:end));
pause(1);
robot.setOperatingMode("p"); % v
pause(1);
tic; % Start timer
% Go to each vertex in order
for i = 2:height(trajectoriesAngles)
    % Velocity Control
    % deltaDis = trajectoriesAngles(i,2:end) - trajectoriesAngles(i-1,2:end);
    % vels = deltaDis./pauseTime;
    % robot.writeVelocities(vels);

    % Position Control
    robot.writeJoints(trajectoriesAngles(i,2:end)); % Write joint values

    % Collect a reading periodically until the setpoint is reached
    while toc < (i-1) * pauseTime
        jointReadings = robot.getJointsReadings();
        dataTime(count) = toc;
        dataJoints(:, :, count) = jointReadings(1:2, :);
        count = count + 1;
    end
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataJoints = dataJoints(:,:,1:count-1);

% Save data to a file
data = struct("time", dataTime, "joints", dataJoints, "traj", trajectoriesAngles);
save("pos_traj_test.mat", "data");