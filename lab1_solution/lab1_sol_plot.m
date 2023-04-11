% Plot all joint angles in 4x1 figure
figure

% Read data from csv
data = readmatrix("data.csv");
timeData = data(:,1);
travelTime = timeData(end);
jointData = data(:,2:end);

% Make four subplots for each joint profile
hold on
for i=1:4
    % Joint i
    subplot(4,1,i); % Creates subplot
    plot(timeData,jointData(:,i),"LineWidth",3) % Plots data

    % Formatting and Labels
    xlim([0, travelTime]);
    ylim([-5, 50]);
    title("Joint " + i + " Angle Position (deg) vs. Time (s)")
    xlabel("Time (s)")
    ylabel("Angle Position (deg)")
    set(gca, "FontSize", 24)
end
hold off

% Plot histogram of time between measurements
figure
histData = timeData(2:end) - timeData(1:end-1); % Get array of time between each measurement
histogram(histData)

% Formatting and Labels
title("Time Between Motor Sensor Readings (s)")
xlabel("Time (s)")
ylabel("Enumeration")
set(gca, "FontSize", 24)

% Determine statistics of measurement time data
analysis = [mean(histData) median(histData) max(histData) min(histData) std(histData)]