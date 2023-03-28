clear
clc
close all

%% Plotting

% Load Data
load('pos_traj_veltest.mat') % pos_traj_postest.mat
time = data.time;
joints = data.joints;
traj = data.traj;
endTime = time(end);
robot = Robot();

for i = 1:length(joints)
    vels = robot.getForwardDiffKinematics(squeeze(joints(1,:,i)), squeeze(joints(2,:,i)))';
    joints(2,:,i) = [vels(1:3) vels(5)];
    joints(1,:,i) = robot.getEEPos(squeeze(joints(1,:,i)));
end

for i = 1:height(traj)
    traj(i,2:end) = robot.getEEPos(traj(i,2:end));
end

% pos, vel, acc subplots for desired trajectory
anglesTime = time;
anglesEndTime = anglesTime(end);

% titles = ["Position (deg)", "Velocity (deg/s)", "Acceleration (deg/s^2)"];
titles = ["Position (mm)", "Velocity (mm/s)", "Acceleration (mm/s^2)"];
units = ["", "/s"];
actualColors = ["red", "green", "blue", "magenta"];
desiredColors = [0.7 0 0; 0 0.7 0; 0 0 0.7; 0.7 0 0.7];

derivativeNum = 1;
xData = traj(:,1);
yData = traj(:,2:end);

% Loop to make as many subplots for position derivatives as desired
for i=1:derivativeNum+1
    figure
    hold on
    yActual = squeeze(joints(i,:,:))';
    for j=1:width(yData)
        plot(time, yActual(:,j),"LineWidth", 3, 'color', actualColors(j));
    end

    for j=1:width(yData)
        plot(xData, yData(:,j),"LineWidth", 3, 'color', desiredColors(j,:),'LineStyle',':');
    end

    % Plot formatting
    xlim([0, xData(end)]);
%     title("Actual and Desired Joint " + titles(i) + " vs. Time (s)")
    title("Actual and Desired End-Effector " + titles(i) + " vs. Time (s)")
    xlabel("Time (s)")
    ylabel(titles(i))
    set(gca, "FontSize", 30)
    grid on
    legend(strcat('Actual EE x (mm',units(i),')'), ...
        strcat('Actual EE y (mm',units(i),')'), ...
        strcat('Actual EE z (mm',units(i),')'), ...
        strcat('Actual EE alpha (deg',units(i),')'), ...
        strcat('Desired EE x (mm',units(i),')'), ...
        strcat('Desired EE y (mm',units(i),')'), ...
        strcat('Desired EE z (mm',units(i),')'), ...
        strcat('Desired EE alpha (deg',units(i),')'))
    
    % Calculate derivative of data (d/dt) for next loop
    yData = (yData(2:end,:) - yData(1:end-1,:)) ./ (xData(2:end,:) - xData(1:end-1,:));
    xData = xData(2:end);
end
