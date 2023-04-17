%% Configurationsclea

% Shutdown configurations
onCleanup(@shutdown); % Shutdown configurations
rosinit; % Initializes ROS

%% Setup robot
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode
model = Model();
%% Program 

jointAngles = [0 0 0 0 ; 45 -15 -60 30 ; -45 0 15 -45 ; 105 -75 60 0];
fkArray = zeros(4, 4, 4);

for i = 1:4
    disp(i)
    fkArray(:,:,i) = robot.getFK(robot.degsToRads(jointAngles(i,:)));
    robot.writeJoints(jointAngles(i,:)); % Write joint values
    disp(jointAngles(i,:))
    tic; % Start timer
    while toc < travelTime
        jointData = robot.getJointsReadingsRadians();
        q = jointData(1,:);
        model.plotArm(q, false);
        pause(0.5);
    end
    pause(10); % time to take photos
end

save('lab2_part7FKData.mat', 'fkArray')

pause(1);
% Shutsdown ROS
function shutdown()
    disp("Shutting Down...");
        rosshutdown;
end