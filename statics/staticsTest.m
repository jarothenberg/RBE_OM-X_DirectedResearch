clear
clc

robot = Robot();
robot.writeMotorState(true);
robot.setOperatingMode('p');
robot.gripper.toggleTorque(false);
travTime = 5;
robot.writeTime(travTime);
robot.writeJoints(0);
pause(travTime);

% tic
while true
    read = robot.getJointsReadings();
    J = robot.getJacobian(read(1,:));
    currents = read(3,:)/1000;
    % baseCurr=(1.8-0.18)./(3.01-0.07).*(torque-0.07)+0.18;
    torque = (currents-0.18).*((3.01-0.07)/(1.8-0.18))+0.07;

    % https://ocw.mit.edu/courses/2-12-introduction-to-robotics-fall-2005/5c160fb678fa75191c399a373c6ce648_chapter6.pdf
    force = inv(J(1:3,1:3)') * torque(1:3)';
    weight = force(3)/4.448

end
robot.writeVelocities([0 0 0 0]);