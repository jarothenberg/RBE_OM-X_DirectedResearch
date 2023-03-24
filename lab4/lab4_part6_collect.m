%% Configurations
clear
clc

travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model();

%% Setup robot
robot.writeMotorState(true); % Write position mode
robot.writeTime(travelTime);

% Define setpoint poses
overheadPos = [0 -atan2d(24,128) atan2d(24,128)-90 0];
stretchedPos = [0 90-atan2d(24,128) atan2d(24,128)-90 0];
homePos = [0 0 0 0];
% Create trajectory of poses
q = overheadPos;
% Convert task space trajectory into a joint space one to follow

%% Program 

% Pre-allocate data
dataTime = zeros(2500, 1);
dataDet = zeros(2500, 1);
dataEePos = zeros(2500, 3);
count = 1;
tolerance = 0.5;
% Send to first vertex to start
robot.writeJoints(homePos);
pause(travelTime);
tic; % Start timer
% Go to each vertex in order
robot.writeJoints(q); % Write joint values
% Collect a reading periodically until the setpoint is reached
while toc < travelTime
    % Read from joints
    jointReadings = robot.getJointsReadings();
    % Collect time data
    dataTime(count) = toc;
    % Calculate end effector pose from joint readings
    eePose = robot.getEEPos(jointReadings(1,:));
    % Add end effector pose to data list
    dataEePos(count,:) = eePose(:,1:3);
    % Calculate Jacobian matrix from joint readings
    J = robot.getJacobian(jointReadings(1,:));
    % Calculate determinant of the first 3x3 of Jacobian
    determinant = det(J(1:3,1:3))
    % Add Jacobian determinant to data list
    dataDet(count,:) = determinant;
    % Checks if robot is too close to singularity
    if abs(dataDet(count,:)) < tolerance
        % Restarts motor states resulting in immediate stopping of motion
        robot.writeMotorState(false);
        robot.writeMotorState(true);
        % Displays error
        error("Approaching singular configuration, stopping")
    end
    % Iterates data counting variable
    count = count + 1;
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataDet = dataDet(1:count-1,:);
dataEePos = dataEePos(1:count-1,:);

% Save data to a file
data = struct("time", dataTime, "eePoses", dataEePos, "Jdets", dataDet);
save("lab4_part6_data.mat", "data");