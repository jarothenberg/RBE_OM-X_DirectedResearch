clear
close all
clc

robot = Robot();
robot.setOperatingMode('p');

robot.writeJoints([0 0 0 0]);
pause(1);

motor = robot.motors(3);
motor.setOperatingMode('c');
kP = 30;
kI = 0.12;
kD = 650;
errorSum = 0;
timePeriod = 0.01;
preErr = 0;
count = 1;
errorCount = 1;
kImemory = 50;
errors = zeros(1,kImemory);

data = zeros(100000, 3);

dataTic = tic;
endTime = 5;

while toc(dataTic) < endTime
    tic;
    targetPose = -90;
    read = motor.getJointReadings()
    curPose = read(1);
    poseErr = targetPose - curPose;
    errors(errorCount) = poseErr;
    errorCount = mod((errorCount + 1),(kImemory-1))+1;
    errorSum = sum(errors);
    errorDer = poseErr - preErr;
    current = poseErr*kP + errorSum*kI + errorDer*kD
    robot.writeCurrent([0 0 current 0]);
    preErr = poseErr;

    data(count,:) = [toc(dataTic) poseErr current];
    count = count + 1;

    while toc < timePeriod
    end
end

motor.writeCurrent(0);
data = data(1:count-1,:);
time = data(:,1);
maxTime = time(end);
error = data(:,2);
current = data(:,3);
figure
subplot(1,2,1)
hold on
plot(time, zeros(size(data(:,1))),"LineWidth", 3)
plot(time, error ,"LineWidth", 3)
xlim([0 maxTime])
title("Position Error vs. Time")
xlabel("Time (s)")
ylabel("Position Error (deg)")
set(gca, "FontSize", 30)
grid on
hold off
subplot(1,2,2)
plot(time, current,"LineWidth", 3)
xlim([0 0.5])
title("Current vs. Time")
xlabel("Time (s)")
ylabel("Current (mA)")
set(gca, "FontSize", 30)
grid on