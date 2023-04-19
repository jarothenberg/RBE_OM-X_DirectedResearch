clear
clc

robot = Robot();
robot.writeMotorState(true);
robot.setOperatingMode('cp');
travTime = 5;
robot.writeTime(travTime);
numObjs = 4;

tMags = zeros(2,numObjs);
tMags(1,:) = 1:1:numObjs;

for i=1:numObjs
    robot.writeGripper(true);
    robot.writeJoints([0 0 -90 0]);
    pause(travTime)
    robot.writeJoints(0)
    pause(travTime*2)
    read = robot.getJointsReadings();
    currents = (read(3,:)/1000)'; % mA to A
    t_0 = (currents).*((3.01-0.07)/(1.8-0.18));
    t_0 = t_0 * 1000;
    robot.writeGripper(false);
    pause(travTime)
    read = robot.getJointsReadings();
    currents = (read(3,:)/1000)'; % mA to A
    t_1 = (currents).*((3.01-0.07)/(1.8-0.18));
    t_1 = t_1 * 1000;
    dt_1 = t_1 - t_0;
    tMags(2,i) = norm(dt_1(2:end));
end

robot.writeGripper(true);
[temp, order] = sort(tMags(2,:),"descend");
disp("Objects in order of weight are: " + num2str(order))

% 
% 
%     pause(5)
%     read = robot.getJointsReadings();
%     currents = (read(3,:)/1000)'; % mA to A
%     t_0 = (currents).*((3.01-0.07)/(1.8-0.18)); % (currents-0.18).*((3.01-0.07)/(1.8-0.18))+0.07
%     t_0 = t_0 * 1000 % N*m to N*mm
%     pause(5)
%     robot.writeGripper(true);
%     pause(5)
%     robot.writeGripper(false);
%     pause(5)
%     read = robot.getJointsReadings();
%     currents = (read(3,:)/1000)';
%     t_1 = (currents).*((3.01-0.07)/(1.8-0.18)); % (currents-0.18).*((3.01-0.07)/(1.8-0.18))+0.07
%     t_1 = t_1*1000
%     dt_m = t_1 - t_0
%     %is the downward direction in the J y?
%     df = [0 0 -1.51 0 0 0]'; %payload
%     %is jacobian in mm? if so maybe some conversion needs to happen?
%     J = robot.getJacobian(read(1,:));
%     dt_c = J'*df
% 
% 
% % tic
% % while true
% %     read = robot.getJointsReadings();
% %     J = robot.getJacobian(read(1,:));
% %     currents = read(3,:)/1000;
% %     % baseCurr=(1.8-0.18)./(3.01-0.07).*(torque-0.07)+0.18;
% %     torque = (currents-0.18).*((3.01-0.07)/(1.8-0.18))+0.07;
% 
% %     % https://ocw.mit.edu/courses/2-12-introduction-to-robotics-fall-2005/5c160fb678fa75191c399a373c6ce648_chapter6.pdf
% %     force = inv(J(1:3,1:3)') * torque(1:3)';
% %     weight = force(3)/4.448;
% 
% % end
% % robot.writeVelocities([0 0 0 0]);