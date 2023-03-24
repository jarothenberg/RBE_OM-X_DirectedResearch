%% Configurations
clear
clc

%% Plotting

% Load Data
load('lab4_part5_data.mat')
% Sets variables from struct for easier plotting
time = data.time;
angles = data.angles;
eeVels = data.eeVels;
% Final time
endTime = time(end);
robot = Robot();
model = Model();

% Creates dynamic plot of robot motion with end effector velocity
% Loops through all frames
for i=1:height(time)-1
    % Plots arm at given point with given end effector velocity
    model.plotArm(angles(i,:),eeVels(i,:),false,"Dynamic Plot")
    % Pause difference in time for accuracy
    pause(time(i+1)-time(i))
end

% Initializes data array
eePose = zeros(size(angles))

% Loops through all angles recorded
for i=1:height(angles)
    % Adds end effector pose to list by calculating from angles
    eePose(i,:) = robot.getEEPos(angles(i,:));
end

% Creates figure
figure
% Assigns parts of data for clarity
xLinVel = eeVels(:,1);
yLinVel = eeVels(:,2);
zLinVel = eeVels(:,3);
xAngVel = eeVels(:,4);
yAngVel = eeVels(:,5);
zAngVel = eeVels(:,6);
% Creates first subplot
subplot(3,1,1)
hold on
% Plots the linear velocites of x, y, and z over time
plot(time, xLinVel,"LineWidth", 3);
plot(time, yLinVel,"LineWidth", 3);
plot(time, zLinVel,"LineWidth", 3);
% Plot formatting
title("Linear Velocity of ee vs. Time")
ylabel("Velocity (mm/s)")
xlabel("time (s)")
legend("xLinVel", "yLinVel", "zLinVel")
set(gca, "FontSize", 35)
hold off
% Creates second subplot
subplot(3,1,2)
hold on
% Plots the angular velocities of x, y, and z over time
plot(time, xAngVel,"LineWidth", 3);
plot(time, yAngVel,"LineWidth", 3);
plot(time, zAngVel,"LineWidth", 3);
% Plot formatting
title("Angular Velocity of ee vs. Time")
ylabel("Velocity (degrees/s)")
xlabel("time (s)")
legend("xAngVel", "yAngVel", "zAngVel")
set(gca, "FontSize", 35)
hold off
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
