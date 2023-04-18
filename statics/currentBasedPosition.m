clc
clear

robot = Robot();
% robot.setOperatingMode('p');
robot.writeMotorState(true);
travTime = 5;
% robot.writeTime(travTime);
mode = 1;

% robot.writeJoints(-15);
% while true
%     if mode == 1
%         op = "p"
%         mode = 0;
%         q = 0;
%     elseif mode == 0
%         op = "cp"
%         mode = 1;
%         q = 20;
%     end
%     robot.setOperatingMode(op);
%     pause(1);
%     robot.writeJoints(-15);
%     pause(travTime);
% end
