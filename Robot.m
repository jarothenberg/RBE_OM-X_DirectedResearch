% Robot class for OpenManipulator-X Robot

classdef Robot

    properties
        motors;
        gripper;
    end

    methods
        function self = Robot()
            motorsNum = 4;
            motorIDs = [11, 12, 13, 14];
            gripperID = 15;
            devices = serialportlist();
            deviceName = convertStringsToChars(devices(1));
            for i=1:motorsNum
                self.motors = [self.motors; DX_XM430_W350(motorIDs(i), deviceName)];
            end
            self.gripper = DX_XM430_W350(gripperID, deviceName);
        end
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            for i = 1:length(self.motors)
                motor = self.motors(i);
                readings(:,i) = motor.getJointReadings();
                disp(readings(1,i))
                fprintf('[ID:%03d] PresPos:%.9f\tPresVel:%.3f\tPresCur:%.3f\n', motor.ID, readings(1,i), readings(2,i), readings(3,i));
            end
        end

        function writeJoints(self, goals)
            for i=1:length(self.motors)
                motor = self.motors(i);
                motor.writePosition(goals(i));
            end
        end

        function toggleTorque(self, enable)
            for i=1:length(self.motors)
                self.motors(i).toggleTorque(enable);
            end
        end
    end
end