clear
clc

%% Plotting

% Load Data
load('lab4_signoff2_data.mat')
% Sets variables from struct for easier plotting
time = data.time;
angles = data.angles;
eeVels = data.eeVels;
% Final time
endTime = time(end);
robot = Robot();

% Initializes data array
eePose = zeros(size(angles));

% Loops through all angles recorded
for i=1:height(angles)
    % Adds end effector pose to list by calculating from angles
    eePose(i,:) = robot.getEEPos(angles(i,:));
end

% Creates figure
figure
% Assigns parts of data for clarity
titles = ["Linear" "Angular"];
ylabels = ["mm" "degs"];
labels = ["Lin" "Ang"];
% Creates first subplot
for i = 1:2
    subplot(3,1,i)
    hold on
    % Plots the linear velocites of x, y, and z over time
    for j = 1:3
        plot(time, eeVels(:,j+3*(i-1)),"LineWidth", 3);
    end
    % Plot formatting
    title(titles(i) + " Velocity of ee vs. Time")
    ylabel("Velocity ("+ylabels(i)+"/s)")
    xlabel("time (s)")
    legend("x"+labels(i)+"Vel", "y"+labels(i)+"Vel", "z"+labels(i)+"Vel")
    set(gca, "FontSize", 35)
    hold off
end
% Creates third subplot
subplot(3,1,3)
% Calculates magnitude of end effector velocity
magnitude = vecnorm(eeVels(:,1:3), 2, 2);
% Plots end effector velocity magnitude over time
plot(time, magnitude,"LineWidth", 3)
% Plot formatting
title("Linear Velocity Magnitude of ee vs. Time")
ylabel("Velocity (mm/s)")
xlabel("time (s)")
set(gca, "FontSize", 35)