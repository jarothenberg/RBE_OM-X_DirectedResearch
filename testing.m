clear
clc

devices = seriallist()
deviceName = devices(1);

motor1 = DX_XM430_W350(11, convertStringsToChars(deviceName));
motor2 = DX_XM430_W350(12, convertStringsToChars(deviceName));
motor3 = DX_XM430_W350(13, convertStringsToChars(deviceName));
motor4 = DX_XM430_W350(14, convertStringsToChars(deviceName));
motor5 = DX_XM430_W350(15, convertStringsToChars(deviceName));

motors = [motor1 motor2 motor3 motor4 motor5];
motor1.toggleTorque(true);

q = [-10 -10 -10 -10 0];

for i=1:length(motors)
    motor = motors(i);
    motor.toggleTorque(true);
    motor.writePosition(q(i));
end

while true
    for i = 1:length(motors)
        motor = motors(i);
        readings = motor.getJointReadings();
        fprintf('[ID:%03d] PresPos:%03d\tPresVel:%03d\tPresCur:%03d\n', motor.ID, readings(1), readings(2), readings(3));
    end
end