clear
clc

robot = Robot();
motor = robot.motors(1);

motor.toggleTorque(false);

pause(2)

motor.setOperatingMode("c");
robot.toggleTorque(true);

pause(2)

currents = [1500];
travelTime = 1;

for i=1:1
    disp(i)

    current = currents(i);
    
    motor.writeCurrent(current);

    tic
    while (toc < travelTime)
        read = motor.getJointReadings()
        rpm = read(2)/6
    end
    
    motor.writeCurrent(0)
    % Send to -90
    motor.writeCurrent(-current);
    pause(5);
    motor.writeCurrent(0);
end
