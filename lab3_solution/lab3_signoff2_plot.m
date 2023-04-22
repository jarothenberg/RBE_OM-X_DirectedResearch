clear
clc

%% Configurations

% Load data
load('lab3_part3_data.mat')
time = data.time;
angles = data.angles;
eePoses = data.eePos;
endTime = time(end);
vertexAngles = data.vertexAngles;
robot = Robot();
model = Model();

% Plot stick models of the robot at each of the vertex positions of
% triangle
figure
for i = 1:3
    subplot(3,1,i)
    model.plotArm(robot.degsToRads(vertexAngles(i,:)), true, "Stick Model Position " + int2str(i))
    set(gca, "FontSize", 25)
    lines = findobj(gca, "Type", "Line")
    set(lines, "LineWidth", 2);
end

% Plot end-effector pose vs time.
figure
hold on
% Joint 1
plot(time,eePoses(:,1),"LineWidth",3)
% Joint 2
plot(time,eePoses(:,2),"LineWidth",3)
% Joint 3
plot(time,eePoses(:,3),"LineWidth",3)
% Joint 4
plot(time,eePoses(:,4),"LineWidth",3)

% Plot formatting
xlim([0, endTime]);
title("End Effector Pose vs. Time (s)")
xlabel("Time (s)")
ylabel("Pose")
set(gca, "FontSize", 50)
legend('X (mm)','Y (mm)','Z (mm)','ϕ (degs)')
grid on
hold off

% Plot end-effector path in 3D
figure
plot3(eePoses(:,1),eePoses(:,2),eePoses(:,3),"LineWidth",3)
title("End Effector Trajectory")
xlabel("x (mm)")
ylabel("y (mm)")
zlabel("z (mm)")
set(gca, "FontSize", 50)
grid on

% Plot joint angles vs time
figure
hold on
% Joint 1
plot(time,angles(:,1),"LineWidth",3)
% Joint 2
plot(time,angles(:,2),"LineWidth",3)
% Joint 3
plot(time,angles(:,3),"LineWidth",3)
% Joint 4
plot(time,angles(:,4),"LineWidth",3)

% Plot formatting
xlim([0, endTime]);
title("Joint Angles Position (deg) vs. Time (s)")
xlabel("Time (s)")
ylabel("Angle Position (deg)")
set(gca, "FontSize", 50)
legend('Joint 1','Joint 2','Joint 3','Joint 4')
grid on
hold off
