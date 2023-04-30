%% Setup robot
% travelTime = 10; % Defines the travel time
travelTime = 2; % For other part of lab
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time

%% Program 
robot.writeJoints(0); % Write joints to zero position
pause(travelTime); % Wait for trajectory completion

baseWayPoints = [45]; % Define base waypoints

% Pre-allocate data to optimize for speed
data = zeros(100000,5);
dataNum = 0;

% Record time and joint angles throughout motion
for baseWayPoint = baseWayPoints % Iterate through waypoints
    robot.writeJoints([baseWayPoint, 0, 0, 0]); % Write joint values

    tic; % Start timer
    while toc < travelTime % read as fast as possible
        dataNum = dataNum + 1;
        jointsReadings = robot.getJointsReadings();
        data(dataNum,:) = [toc jointsReadings(1,:)]; % Read time and joint values
    end
end

% remove unused space and write to csv
data = data(1:dataNum,:);
writematrix(data, "data2.csv");