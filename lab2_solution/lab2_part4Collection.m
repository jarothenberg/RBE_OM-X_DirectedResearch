%% Setup robot
travelTime = 4; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
%% Program 
robot.writeJoints([0 0 0 0])
pause(travelTime)

T = zeros(4, 4, 5);

for i = 1:5
    robot.writeJoints([i * 10, i * 2, i * 2, i * 2]); % Write joint values
    pause(travelTime + 1);

    robot.writeJoints([0, 0, 0, 0]); % Write joint values
    pause(travelTime + 1); % Wait for trajectory completion

    T(:,:,i) = robot.getCurrentFK();
end

save('lab2_data.mat', "T") % Save T to file for plotting script to read from