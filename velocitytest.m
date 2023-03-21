clear
clc

robot = Robot();
motor = robot.motors(1);

robot.toggleTorque(false)

pause(2)

robot.setOperatingMode("v");

pause(2)

robot.toggleTorque(true);

speeds = [6 12 18 24 30];
travelTime = 5;

for i=1:5
    disp(i)
    start = motor.getJointReadings();
    start = start(1);

    speed = speeds(i);
    
    motor.writeVelocity(speed);

    tic
    while (toc < travelTime)
        motor.getJointReadings()
    end
    
    motor.writeVelocity(0)
    % Send to -90
    motor.writeVelocity(-speed);
    pause(5);
    motor.writeVelocity(0);
end
