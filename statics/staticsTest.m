clear
clc

robot = Robot();
robot.writeMotorState(true);
robot.writeMode('p');
travTime = 5;
robot.writeTime(travTime);
numObjs = 4;

tMags = zeros(2,numObjs);
tMags(1,:) = 1:1:numObjs;

for i=1:numObjs
    disp("resetting position")
    robot.writeGripper(true);
    robot.writeJoints([0 0 -10 0]);
    pause(travTime)
    robot.writeJoints(0)
    pause(travTime*2)
    disp("reading t_0")
    read = robot.getJointsReadings();
    currents = (read(3,:)/1000)'; % mA to A
    t_0 = (currents).*((3.01-0.07)/(1.8-0.18));
    t_0 = t_0 * 1000;
    disp("place object")
    pause(travTime*2)
    robot.writeGripper(false);
    pause(travTime)
    disp("reading t_1")
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