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

% pos, vel, acc subplots for desired trajectory
anglesTime = time;
anglesEndTime = anglesTime(end);

titles = ["Position (deg)", "Velocity (deg/s)", "Acceleration (deg/s^2)"];
units = ["", "/s"];

derivativeNum = 1;
xData = traj(:,1);
yData = traj(:,2:end);

% Loop to make as many subplots for position derivatives as desired
for i=1:derivativeNum+1
    figure
    hold on
    yActual = squeeze(joints(i,:,:))';
    for j=1:width(yData)
        plot(time, yActual(:,j),"LineWidth", 3)
    end

    for j=1:width(yData)
        plot(xData, yData(:,j),"LineWidth", 3)
    end

    % Plot formatting
    xlim([0, xData(end)]);
    title("Actual and Desired Joint " + titles(i) + " vs. Time (s)")
    xlabel("Time (s)")
    ylabel(titles(i))
    set(gca, "FontSize", 30)
    grid on
%     unitDivision = strcat('/s^', int2str(i-1));
    legend(strcat('Actual Joint 1 (deg',units(i),')'), ...
        strcat('Actual Joint 2 (deg',units(i),')'), ...
        strcat('Actual Joint 3 (deg',units(i),')'), ...
        strcat('Actual Joint 4 (deg',units(i),')'), ...
        strcat('Desired Joint 1 (deg',units(i),')'), ...
        strcat('Desired Joint 2 (deg',units(i),')'), ...
        strcat('Desired Joint 3 (deg',units(i),')'), ...
        strcat('Desired Joint 4 (deg',units(i),')'))
    hold off


    
    % Calculate derivative of data (d/dt) for next loop
    yData = (yData(2:end,:) - yData(1:end-1,:)) ./ (xData(2:end,:) - xData(1:end-1,:));
    xData = xData(2:end);
end
