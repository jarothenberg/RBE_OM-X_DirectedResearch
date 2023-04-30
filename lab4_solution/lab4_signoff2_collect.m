%% Configurations
clear
clc

travelTime = 5; % Defines the travel time
robot = Robot(); % Creates robot object
model = Model(); % Creates model object

% Define setpoint poses
eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45 ; 25 -100 150 -60];

% Create trajectory of poses
tj = TrajPlanner(eePoses);
trajectories = tj.getQuinticTraj(travelTime,0);

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
robot.writeMotorState(true);
robot.writeTime(pauseTime);

%% Program 

% Pre-allocate data
dataTime = zeros(2500, 1);
dataAngles = zeros(2500, 4);
dataPDot = zeros(2500, 6);
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
        % Read from joints
        read = robot.getJointsReadings();
        q = read(1,:);
        qDot = read(2,:);
        % Collect time data
        dataTime(count) = toc;
        % Records current angles in data list
        dataAngles(count, :) = q;
        % Calculates end effector velocity
        pDot = robot.getForwardDiffKinematics(q,qDot)';
        % Assigns end effector velocity to data list
        dataPDot(count, :) = pDot;
        % Increments data count variable
        count = count + 1;
        model.plotArm(q,pDot,false,"Dynamic Plot")
        pause(0)
    end
end

% Trim unused space in data
dataTime = dataTime(1:count-1,:);
dataAngles = dataAngles(1:count-1,:);
dataPDot = dataPDot(1:count-1,:);

% Save data to a file
data = struct("time", dataTime, "angles", dataAngles, "eeVels", dataPDot);
save("lab4_signoff2_data.mat", "data");
