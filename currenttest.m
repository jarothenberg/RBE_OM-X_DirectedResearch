clear
clc

robot = Robot();
motor = robot.motors(1);

motor.toggleTorque(false);
motor.setOperatingMode("c");
motor.toggleTorque(true);


current = 2.5;
motor.writeCurrent(current)
pause(2.5)
motor.writeCurrent(0)
pause(2.5)
motor.writeCurrent(-current);
pause(2.5)

motor.writePosition(-90);

pause(1)

motor.toggleTorque(false);
motor.setOperatingMode("v");
robot.toggleTorque(true);

pause(1)

speeds = [10 20 30 40 50];

for i=1:5
    disp(i)
    start = motor.getJointReadings();
    start = start(1);

    speed = speeds(i);
    
    pause(1);
    
    tic;
    while (toc < 5)
        motor.writeVelocity(speed);
    end
    
    motor.writeVelocity(0);
    ending = motor.getJointReadings();
    ending = ending(1);
    disp(ending-start)

    % Send to -90
    motor.writeVelocity(-speed)
    pause(5);
    motor.writeVelocity(0);
end
