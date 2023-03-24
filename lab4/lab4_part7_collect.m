%% Configurations
clear
clc

robot = Robot(); % Creates robot object

% Define setpoint poses
eePoses = [25 -100 150 0; 150 80 300 0; 250 -115 75 0 ; 25 -100 150 0];
% eePoses = [274 0 204 0 ; 200 0 300 0];
angles = eePoses;

% Converts all joint angles to angles from IK
for i = 1:height(angles)
    try
        angles(i,:) = robot.getIK(eePoses(i,:));
    catch
        error("End-Effector Pose Unreachable")
    end
end

%% Setup robot
robot.writeMotorState(true);

%% Program 

% Pre-allocate data
dataTime = zeros(2500, 1);
dataEePos = zeros(2500, 4);

tolerance = 30;
count = 1;
velocity = 25; %mm/s
% Send to first vertex to start
robot.writeJoints(angles(1,:));
pause(2);
tic;

for i = 2:4 % Loops through all positions
    % Instansiates distance so the while actually runs
    distance = [100 100 100 100];
    % Loops until within tolerance of goal
    while norm(distance) > tolerance
        % Measures current joint readings
        jointReadings = robot.getJointsReadings();
        % Calculates distance of ee from goal
        distance = eePoses(i,:) - robot.getEEPos(jointReadings(1,:));
        % Normalizes distance into vector
        dirVector = distance(:,1:3)./norm(distance(:,1:3));
        % Creates velocity vector by multiplying distance vector by vel
        velVector = (dirVector.*velocity)';
        % Creates Jacobian from current joint angles
        J = robot.getJacobian(jointReadings(1,:));
        % Calculates the pseudoinverse of linear Jacobian
        jpInv = pinv(J(1:3,:));
        % Calculates instantaneous velocity of joints
        jointVels = (jpInv*velVector)';
        % Writes velocity to joints by adding to current angle readings
        anglesWrite = jointReadings(1,:) + jointVels;
        % Writes angles to robot
        robot.writeJoints(anglesWrite);
        % Stores data for later plotting and analysis
        dataTime(count) = toc;
        dataEePos(count, :) = robot.getEEPos(jointReadings(1,:));
        count = count + 1;
    end
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataEePos = dataEePos(1:count-1,:);

% Save data to a file
data = struct("time", dataTime, "eePoses", dataEePos);
save("lab4_part7_data.mat", "data");