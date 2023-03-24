%% Configurations
clear
clc

% Load Data
load('lab4_part6_data.mat')
% Sets variables from struct for easier plotting
time = data.time;
eePoses = data.eePoses;
jDets = data.Jdets;
% Final time
endTime = time(end);

% Creates figure
figure
% Creates first subplot
subplot(2,1,1)
% Plots end effector path in 3d space
plot3(eePoses(:,1),eePoses(:,2),eePoses(:,3),"LineWidth", 3)
% Formatting plot
xlabel("x (mm)")
ylabel("y (mm)")
zlabel("z (mm)")
title("EE Path in Space From Base Pose to Singular Config")
set(gca, "FontSize", 50)
xlim([250 400]) 
ylim([-50 50])
zlim([0 250])
% Creates second subplot
subplot(2,1,2)
% Plots determinants over time
plot(time,jDets,"LineWidth", 3)
% Formatting plot
xlabel("time (s)")
ylabel("Determinant of Linear  Velocity")
title("Determinant of Linear Velocity over Time")
set(gca, "FontSize", 50)