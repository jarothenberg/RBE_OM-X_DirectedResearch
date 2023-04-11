clear;
clc;

updateTime = 0.05;
robot = Robot(); % Creates robot object

%% Setup robot
robot.writeMotorState(true); % Write position mode
robot.setOperatingMode('p');
robot.writeTime(0);
robot.writeJoints(0);
pause(2.5);
robot.motors(1).setOperatingMode('c');
% robot.motors(2).setOperatingMode('c');
% robot.motors(4).setOperatingMode('c');

% PID constants
kP = [1 20 23 8]
kI = [0.3 1.75 0.6 0.35]
kD = [15 50 25 5] 
kImemory = 10;

data = zeros(100000, 9);
count = 1;

totalTic = tic;

% Go to each vertex in order

errorSums = zeros(1,4);
preErr = zeros(1,4);
errorCount = 1;
errors = zeros(kImemory,4);
errorsDer = zeros(1,4);
startTic = tic;

while toc(startTic) < 5
    updateTic = tic;

    % Current output from PID with joint angles errors
    targetAngles = [45 -15 -15 -15];
    read = robot.getJointsReadings();
    curAngles = read(1,:);
    
    anglesErr = targetAngles - curAngles;
    errors(errorCount, :) = anglesErr;
    errorCount = mod((errorCount + 1),(kImemory-1))+1;
    errorSums = sum(errors);
    errorDer = anglesErr - preErr;
    
    currents = anglesErr.*kP + errorSums.*kI + errorDer.*kD;
%         robot.writeJoints(targetAngles);
    robot.writeCurrent(currents);
    % disp(current);
    data(count,:) = [toc(totalTic) targetAngles curAngles];
    count = count + 1;
    
    preErr = anglesErr;
%         measureTime = tic;
    % Collect a reading periodically until the setpoint is reached
    while toc(updateTic) < updateTime
        %disp(toc)
        % jointReadings = robot.getJointsReadings();
        % dataTime(count) = toc;
        % dataEePoses(count, :) = robot.getEEPos(jointReadings(1,:));
        % count = count + 1;
    end
%         disp(toc(measureTime))
end

robot.setOperatingMode('v');
robot.writeVelocities(0);

data = data(1:count-1,:);
time = data(:,1);
maxTime = time(end);
angleErr = data(:,2:5) - data(:,6:end);
figure
hold on
plot(time, zeros(size(data(:,1))),"LineWidth", 3)
for i=1:4
    plot(time, angleErr(:,i),"LineWidth", 3)
end
xlim([0 maxTime])
title("Position Error vs. Time")
xlabel("Time (s)")
ylabel("Position Error (deg)")
legend("0", "Joint 1", "Joint 2", "Joint 3", "Joint 4")
set(gca, "FontSize", 30)
grid on
hold off

colors = ["red","green","blue","magenta"];
figure
hold on
for i=1:4
    plot(time, data(:,i+1), "Color",colors(i),"LineStyle","-","LineWidth", 3);
    plot(time, data(:,i+5), "Color",colors(i),"LineStyle",":","LineWidth", 3);
end
xlim([0 maxTime])
title("Joint Position vs. Time")
xlabel("Time (s)")
ylabel("Position (deg)")
legend('J1 Ideal', 'J1 Actual', 'J2 Ideal', 'J2 Actual', 'J3 Ideal', 'J3 Actual', 'J4 Ideal', 'J4 Actual');
set(gca, "FontSize", 30)
hold off