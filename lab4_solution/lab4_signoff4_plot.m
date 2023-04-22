clear
clc

%% Plotting

% Load Data
load('lab4_signoff4_data.mat')
% Sets variables from struct for easier plotting
time = data.time;
eePoses = data.eePoses;
% Final time
endTime = time(end);
% List of derivative titles for graphs
titles = ["Position", "Velocity", "Acceleration"];

% Number of derivates to calculate and plot
derivativeNum = 2;

% Setting of generic x and y
xData = time;
yData = eePoses;

% Creates figure
figure
% Loop to make as many subplots for position derivatives as desired
for i=1:derivativeNum+1
    subplot(derivativeNum+1,1,i)
    hold on
    % Plots x and y data for all y datasets
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
    legend(strcat('X (mm',unitDivision,')'),strcat('Y (mm',unitDivision,')'),strcat('Z (mm',unitDivision,')'),strcat('ϕ (degs',unitDivision,')'), ,strcat(' (mm',unitDivision,')'))
    hold off
    
    % Calculate derivative of data (d/dt) for next loop
    yData = (yData(2:end,:) - yData(1:end-1,:)) ./ (xData(2:end,:) - xData(1:end-1,:));
    xData = xData(2:end);
end