clear
clc

robot = Robot();
robot.writeMotorState(true);
robot.setOperatingMode('p');
travTime = 5;
robot.writeTime(travTime);
q = [0 0 0 0];
robot.writeJoints(q);
pause(travTime);

robot.setOperatingMode('v');
qDotGiven = [-5 -5 -5 0];
robot.writeVelocities(qDotGiven);
tic
while (toc < travTime)
    read = robot.getJointsReadings();
    qRead = read(1,:);
    J = robot.getJacobian(qRead);
    qDotRead = read(2,:)
    pDotMeasure = J*qDotRead';
    qDotCalc = inv(J(1:3,1:3))*pDotMeasure(1:3)
end
robot.writeVelocities([0 0 0 0]);