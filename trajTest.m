clear
clc

eePoses = [25 -100 150 -60; 150 80 300 0; 250 -115 75 -45; 25 -100 150 -60];

robot = Robot();

robot.setOperatingMode('p');
robot.writeMotorState(true);
robot.writeTime(0);
robot.writeJoints(robot.getIK(eePoses(1,:)));
pause(1)
robot.setOperatingMode('v');
robot.writeMotorState(true);

travelTime = 5;
pointsNum = 100;

jointsAngles = zeros(4,4);
for i = 1:4
    jointsAngles(i,:) = robot.getIK(eePoses(i,:));
end

tj = TrajPlanner(jointsAngles);

traj = tj.getQuinticTraj(travelTime, pointsNum);

for i = 1:(height(traj)-1)
    deltaDis = traj(i+1,2:end) - traj(i,2:end);
    deltaTime = traj(i+1,1) - traj(i,1);
    vels = deltaDis./deltaTime;
    robot.writeVelocities(vels);
    pause(deltaTime);
end

robot.writeVelocities([0 0 0 0]);