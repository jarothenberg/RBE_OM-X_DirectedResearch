clear;
clc;

travelTime = 10; % Defines the travel time
updateTime = 0.0;
robot = Robot(); % Creates robot object

% Define setpoint poses, convert to angles for joint space trajectory
eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45 ; 25 -100 150 -60];
for i = 1:height(eePoses)
    jointAngles(i,:) = robot.getIK(eePoses(i,:));
end

traj = TrajPlanner(jointAngles);

degreeN = 5;

coeffArry = zeros((height(jointAngles)-1), degreeN+1, 4);
for i = 1:4 % loops through joints
    for j = 1:(height(jointAngles)-1) % Loops through each setPoint
        % Calculates coeffs for given setPoint for given joint
        coeff = traj.calcNCoeff(0, travelTime, jointAngles(j,i), jointAngles(j+1,i), degreeN)';
        coeffArry(j,:,i) = coeff(:);
    end
end

%% Setup robot
robot.writeMotorState(true); % Write position mode
robot.setOperatingMode('p');
robot.writeTime(0);
robot.writeJoints(jointAngles(1,:));
pause(2.5);
robot.setOperatingMode('c');
% robot.motors(2).setOperatingMode('c');
% robot.motors(4).setOperatingMode('c');

% PID constants
kP = [4.55 20 23 8]
kI = [0.12 1 0.6 0.35]
kD = [25 50 25 5]
kImemory = 100000;

data = zeros(100000, 17);
count = 1;

totalTic = tic;

% Go to each vertex in order
for pathNum=1:(height(jointAngles)-1)
    errorSums = zeros(1,4);
    preErr = zeros(1,4);
    errorCount = 1;
    errors = zeros(kImemory,4);
    errorsDer = zeros(1,4);
    
    startTime = tic;
    while toc(startTime) < travelTime
        updateTic = tic;

        % Current output from PID with joint angles errors
        targetAngles = calcAngles(pathNum, coeffArry, toc(startTime));
        forward = calcBaseCurr(pathNum, coeffArry, toc(startTime), updateTime);
        read = robot.getJointsReadings();
        curAngles = read(1,:);
        curCurr = read(3,:);
        
        anglesErr = targetAngles - curAngles;
        errors(errorCount, :) = anglesErr;
        errorCount = mod((errorCount + 1),(kImemory-1))+1;
        errorSums = sum(errors);
        errorDer = anglesErr - preErr;
        
        currents = anglesErr.*kP + errorSums.*kI + errorDer.*kD;
        robot.writeJoints(targetAngles);
        robot.writeCurrent(currents);
        % disp(current);
        data(count,:) = [toc(totalTic) targetAngles curAngles currents curCurr];
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
end

robot.setOperatingMode('v');
robot.writeVelocities(0);

data = data(1:count-1,:);
time = data(:,1);
maxTime = time(end);
angleErr = data(:,2:5) - data(:,6:9);
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

colors = ["red","green","blue","magenta"];
figure
hold on
for i=1:4
    plot(time, data(:,i+9), "Color",colors(i),"LineStyle","-","LineWidth", 3);
    plot(time, data(:,i+13), "Color",colors(i),"LineStyle",":","LineWidth", 3);
end
xlim([0 maxTime])
title("Joint Current vs. Time")
xlabel("Time (s)")
ylabel("Current (mA)")
legend('J1 PID Curr', 'J1 Act Curr', 'J2 PID Curr', 'J2 Act Curr', 'J3 PID Curr', 'J3 Act Curr', 'J4 PID Curr', 'J4 Act Curr');
set(gca, "FontSize", 30)
hold off

function angles = calcAngles(pathNum, coeffMat, time)
    angles = zeros(1,4);
    for j=1:4
        coeffs = coeffMat(pathNum,:,j);
        for i=1:length(coeffs)
            angles(j) = angles(j) + coeffs(i)*time^(i-1);
        end
    end
end

function baseCurr = calcBaseCurr(pathNum, coeffMat, time, timeDiff)
    startAngles = calcAngles(pathNum, coeffMat, time - timeDiff/2);
    endAngles = calcAngles(pathNum, coeffMat, time + timeDiff/2);
    vels = (endAngles - startAngles) ./ timeDiff;
    rpm = vels./6;
    torque = (rpm-44).*(3.01-0.07)./(6-44)+0.07;
%     torque = -(rpm+44).*(3.01-0.07)./(6-44)+0.07;
    baseCurr=(1.8-0.18)./(3.01-0.07).*(torque-0.07)+0.18;
    baseCurr = baseCurr.*sign(vels)*1000;
end