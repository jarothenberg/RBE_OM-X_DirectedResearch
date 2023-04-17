clear
clc
load('lab2_part4_data.mat') % Read from collection script saved file

% Calculate theoretical End Effector Position
robot = Robot();
theoryPos = robot.getFK([0,0,0,0]); % Get FK for zero position
theoryPos = theoryPos(1:3,4); % Take translation portion to get end effector pos

disp("Theoretical End Effector Position [mm]")
disp(theoryPos')

% Calculate Stats and Errors
T = reshape(T(1:3,4,:), 3, 5);

avgTrialPos = mean(T,2);
disp("Observed Average End Effector Position [mm]")
disp(avgTrialPos')

stdTrialPos = std(T,0,2);
disp("Observed Standard Deviation of Effector Position [mm]")
disp(stdTrialPos')

rmseTrial = zeros(1,5);
for i = 1:5
    rmseTrial(i) = sqrt(mean((theoryPos'-T(:,i)').^2));
end
disp("RMSE of Effector Position between Theoretical and Observed for each Trial[mm]")
disp(rmseTrial)

% Plot End Effector Positions for each of five trials
colors = ['r','g','b','c','m'];

figure

% Plot actual End Effector Position
plot3(theoryPos(1,:),theoryPos(2,:),theoryPos(3,:), ...
    "Color",'k',"Marker","o","MarkerFaceColor",'k')

hold on

% Plot Trial End Effector Positions
for i = 1:5
    plot3(T(1,i),T(2,i),T(3,i),"Color",colors(i),"Marker","o","MarkerFaceColor",colors(i))
end

% Plot Formatting
legend('Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Theoretical')
grid on

xlim([273 280]) 
ylim([-5 5])
zlim([185 210])

title("Robot End Effector Positions (wrt base frame)")
xlabel('x [mm]')
ylabel('y [mm]')
zlabel('z [mm]')

hold off