clear
clc

travelTime = 5; % Defines the travel time
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

tolerance = 5;
count = 1;
velocity = 250; %mm/s

robot.writeJoints(angles(1,:));
pause(travelTime+1);

robot.writeMode("v");

tic;
for i = 2:4 % Loops through all positions
    % Instansiates distance so the while actually runs
    distance = [100 100 100 100];


    % Loops until within tolerance of goal
    while norm(distance) > tolerance
        read = robot.getJointsReadings();
        q = read(1,:);
        % Calculates distance of ee from goal
        eePose = robot.getEEPos(q);

        distance = eePoses(i,1:3) - eePose(1:3);
        % Normalizes distance into vector
        dirVector = distance./norm(distance);

        % Creates velocity vector by multiplying distance vector by vel
        velVector = (dirVector.*velocity)';

        % Creates Jacobian from current joint angles
        J = robot.getJacobian(q);
        
        % Calculates instantaneous velocity of joints
        jpInv = pinv(J(1:3,:));
        jointVels = (jpInv*velVector)';

        robot.writeVelocities(jointVels);

        dataTime(count) = toc;
        dataEePos(count, :) = robot.getEEPos(q);
        count = count + 1;
        norm(distance)
    end

    robot.writeVelocities(0);
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataEePos = dataEePos(1:count-1,:);

% Save data to a file
data = struct("time", dataTime, "eePoses", dataEePos);
save("lab4_signoff4_data.mat", "data");