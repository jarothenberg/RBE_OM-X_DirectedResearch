clear
clc

%% Configurations

% Shutdown configurations
onCleanup(@shutdown); % Shutdown configurations
rosinit; % Initializes ROS
%%

robot = Robot();
load('lab2_part8_data.mat')
dataTitles = ["XZ" "XY"];
planeTitles = ["Z" "Y"];

data = saveData.data;
travelTime = data(end,1);
timeData = data(:,1);

% Plot all joint angles in 4x1 figure
figure
hold on
% Joint 1
plot(timeData,data(:,2))
% Joint 2
plot(timeData,data(:,3))
% Joint 3
plot(timeData,data(:,4))
% Joint 4
plot(timeData,data(:,5))
xlim([0, travelTime]);
title("Joint Angles Position (deg) vs. Time (s) for Travel in XZ Plane")
xlabel("Time (s)")
ylabel("Angle Position (deg)")
set(gca, "FontSize", 24)
legend('Joint 1','Joint 2','Joint 3','Joint 4')
grid on
hold off

figure
hold on
plot(timeData,data(:,6))
plot(timeData, data(:,8))
xlim([0, travelTime]);
title("End Effector Position (mm) vs. Time (s) for Travel along XZ Axes")
xlabel("Time (s)")
ylabel("End Effector Position (mm)")
set(gca, "FontSize", 24)
legend('Travel in X', 'Travel in Z')
grid on
hold off

for i = 1:2
    pointsData = saveData.qs;
    
    eePoints = zeros(height(pointsData), 3);
    for j = 1:height(pointsData)
        eePoints(j,:) = robot.getEEPos(robot.degsToRads(pointsData(j,:)));
    end

    figure
    hold on
    plot(data(:,6), data(:,8 - (i - 1)))
    if i == 1
            plot(eePoints(:,1), eePoints(:,3), "ro", "MarkerSize", 25)
    end
    title("End Effector Position (mm) for Travel in " + dataTitles(i) + " Plane")
    xlabel("X (mm)")
    ylabel(planeTitles(i) + " (mm)")
    set(gca, "FontSize", 24)
    grid on
    hold off
end

% Shutsdown ROS
function shutdown()
    disp("Shutting Down...");
        rosshutdown;
end
