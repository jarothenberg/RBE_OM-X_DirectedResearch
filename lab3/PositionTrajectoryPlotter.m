clear
clc
close all

%% Plotting

% Load Data
load('pos_traj_test.mat')
time = data.time;
joints = data.joints;
traj = data.traj;
endTime = time(end);
robot = Robot();
model = Model();

% pos, vel, acc subplots for desired trajectory
anglesTime = time;
anglesEndTime = anglesTime(end);

titles = ["Position (deg)", "Velocity (deg/s)", "Acceleration (deg/s^2)"];

derivativeNum = 2;
xData = anglesTime;
yData = reshape(joints(1,:,:),4,188)';

% Loop to make as many subplots for position derivatives as desired
for i=1:derivativeNum+1
    figure
    hold on
    for j=1:width(yData)
        plot(xData, yData(:,j),"LineWidth", 3)
    end

    % Plot formatting
    xlim([0, xData(end)]);
    title("Actual Trajectory Joint " + titles(i) + " vs. Time (s)")
    xlabel("Time (s)")
    ylabel(titles(i))
    set(gca, "FontSize", 30)
    grid on
    unitDivision = strcat('/s^', int2str(i-1));
    legend(strcat('Joint 1 (deg',unitDivision,')'),strcat('Joint 2 (deg',unitDivision,')'),strcat('Joint 3 (deg',unitDivision,')'),strcat('Joint 4 (deg',unitDivision,')'))
    hold off
    
    % Calculate derivative of data (d/dt) for next loop
    yData = (yData(2:end,:) - yData(1:end-1,:)) ./ (xData(2:end,:) - xData(1:end-1,:));
    xData = xData(2:end);
end
