clear
clc

%% Configurations

% Shutdown configurations
onCleanup(@shutdown); % Shutdown configurations
rosinit; % Initializes ROS
%% Plotting

% Load data
load('lab3_part6_data.mat')
time = data.time;
eePoses = data.eePose;
angles = data.angles;
endTime = time(end);
% Filter out spikes, ee pose x pos gets above a certain value
time = time(eePoses(:,1)<271.9);
eePoses = eePoses(eePoses(:,1)<271.9,:);
robot = Robot();
model = Model();

% pos, vel, acc subplots
anglesTime = angles(:,1);
anglesEndTime = anglesTime(end);

titles = ["Position", "Velocity", "Acceleration", "Jerk", "Snap", "Crackle", "Pop"];

derivativeNum = 2;
xData = time;
yData = eePoses;
figure
% Loop to make as many subplots for position derivatives as desired
for i=1:derivativeNum+1
    subplot(derivativeNum+1,1,i)
    hold on
    for j=1:width(yData)
        plot(xData, yData(:,j),"LineWidth", 3)
    end

    % Plot formatting
    xlim([0, xData(end)]);
    title("End Effector " + titles(i) + " vs. Time (s)")
    xlabel("Time (s)")
    ylabel(titles(i))
    set(gca, "FontSize", 50)
    grid on
    unitDivision = strcat('/s^', int2str(i-1));
    legend(strcat('X (mm',unitDivision,')'),strcat('Y (mm',unitDivision,')'),strcat('Z (mm',unitDivision,')'),strcat('ϕ (degs',unitDivision,')'))
    hold off
    
    % Calculate derivative of data (d/dt) for next loop
    yData = (yData(2:end,:) - yData(1:end-1,:)) ./ (xData(2:end,:) - xData(1:end-1,:));
    xData = xData(2:end);
end

% Plot triangle path in 3D
figure
plot3(eePoses(:,1),eePoses(:,2),eePoses(:,3),"LineWidth",3)
title("End Effector Trajectory")
xlabel("x (mm)")
ylabel("y (mm)")
zlabel("z (mm)")
set(gca, "FontSize", 50)
grid on

% Plot angles over time
figure
hold on
for i=2:5
    % Joint i
    plot(anglesTime,angles(:,i),"LineWidth",3)
end

% Plot formatting
xlim([0, anglesEndTime]);
title("Joint Angles Position (deg) vs. Time (s)")
xlabel("Time (s)")
ylabel("Angle Position (deg)")
set(gca, "FontSize", 50)
legend('Joint 1','Joint 2','Joint 3','Joint 4')
grid on
hold off

% Shutsdown ROS
function shutdown()
    disp("Shutting Down...");
        rosshutdown;
end
