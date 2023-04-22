clear
clc

%% Plotting

% Load Data
load('lab4_signoff3_data.mat')
% Sets variables from struct for easier plotting
time = data.time;
eePoses = data.eePoses;
manip = data.manip;
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
mins = min(eePoses)
maxs = max(eePoses)
xlim([mins(1) maxs(1)])
ylim([mins(2) maxs(2)])
zlim([mins(3) maxs(3)])
grid on
% Creates second subplot
subplot(2,1,2)
% Plots determinants over time
plot(time,manip,"LineWidth", 3)
% Formatting plot
xlabel("time (s)")
ylabel("manipulability")
title("Manipulability of Arm over Time")
set(gca, "FontSize", 50)