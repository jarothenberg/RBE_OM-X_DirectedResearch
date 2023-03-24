% This script generates a trajectory of an arbitrary degree and plots the 
% theoretical motion.
travelTime = 5; % Defines the travel time

eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45 ; 25 -100 150 -60];
tj = TrajPlanner(eePoses);
trajectories = tj.getNTraj(travelTime,25,23);% N must be odd, 3=cubic, 5=quintic

titles = ["Position", "Velocity", "Acceleration", "Jerk", "Snap", "Crackle", "Pop", "Lock", "Drop", "Shot", "Put"];

% Loop through and create a plot (new figure) for each derivative of
% position desired
derivativeNum = 10;
xData = trajectories(:,1);
yData = trajectories(:,2:end);
for i=1:derivativeNum+1
    
    figure
    hold on
    for j=1:width(yData)
        plot(xData, yData(:,j),"LineWidth", 3)
    end

    % Plot formatting
    xlim([0, xData(end)]);
    if i <= length(titles)
        title("End Effector " + titles(i) + " vs. Time (s)")
        ylabel(titles(i))
    else
        title("End Effector " + int2str(i-1) + " derivative of position vs. Time (s)")
        ylabel(int2str(i-1) + "derivative of position")
    end
    xlabel("Time (s)")
    
    set(gca, "FontSize", 50)
    grid on
    unitDivision = strcat('/s^', int2str(i-1));
    legend(strcat('X (mm',unitDivision,')'),strcat('Y (mm',unitDivision,')'),strcat('Z (mm',unitDivision,')'),strcat('α (degs',unitDivision,')'))
    hold off
    
    % Take derivative of data for the next loop
    yData = (yData(2:end,:) - yData(1:end-1,:)) ./ (xData(2:end,:) - xData(1:end-1,:));
    xData = xData(2:end);
end