% Robot class for OpenManipulator-X Robot

classdef Robot

    properties
        motors;
        gripper;

        % From old Robot class
        mDim; % Stores the robot link dimentions (cm)
        mOtherDim; % Stores extraneous second link dimensions (cm)
        % Forward Kinematics variables  
        mDHTable; % DH Table for the arm. Thetas need to be added to the joint angles first
    end

    methods
        function self = Robot()
            motorsNum = 4;
            motorIDs = [11, 12, 13, 14];
            gripperID = 15;

            try
                devices = serialportlist();
                deviceName = convertStringsToChars(devices(1));
            catch exception
                error("Failed to connect via serial, no devices found.")
            end
            % Find serial port and connect to it
            

            for i=1:motorsNum
                self.motors = [self.motors; DX_XM430_W350(motorIDs(i), deviceName)];
            end
            self.gripper = DX_XM430_W350(gripperID, deviceName);
            self.gripper.setOperatingMode('p');
            self.gripper.toggleTorque(true);

            % From old Robot class
            self.mDim = [77, 130, 124, 126]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            % Forward Kinematics variables
            self.mDHTable = [0 77 0 -90; (asind(24/130) - 90) 0 130 0; (90 - asind(24/130)) 0 124 0; 0 0 126 0];

        end

        % Returns the joint angles that will cause the end-effector to be
        % at the desired pose (x,y,z,phi) based on the inverse kinematics
        % of the arm. If the pose is not possible, this method will throw
        % an error.
        % pose [1x4 double] - The pose (x,y,z,phi) of the end-effector to
        % calculate the corresponding joint angles for.
        function q = getIK(self, pose)
            % Define Links and givens from pose
            L1 = self.mDim(1);
            L2 = self.mDim(2);
            L3 = self.mDim(3);
            L4 = self.mDim(4);

            xe = pose(1);
            ye = pose(2);
            ze = pose(3);
            phi = pose(4); % pitch

            % thetas array with each row corresponding to one of the 
            % two possible solutions (elbow up vs down)
            thetas = zeros([4,2]);
            
            thetas(1,:) = [atan2d(ye,xe) atan2d(ye,xe)];
            re = sqrt(xe^2 + ye^2);

            % Wrist position
            rw = re - L4*cosd(phi);
            zw = ze - L1 - L4*sind(phi);
            dw = sqrt(rw^2 + zw^2);
            
            % Two values for Beta
            cbeta = (L2^2 + L3^2 - dw^2)/(2*L2*L3);
            sbeta = [sqrt(1-(cbeta)^2) -sqrt(1-(cbeta)^2)];
            try
                beta = [atan2d(sbeta(1),cbeta) atan2d(sbeta(2),cbeta)];
            catch
                error("End-Effector Pose Unreachable")
            end
            
            % Constant value of psi
            psi = atand(128/24);
            
            % 180 = psi + beta + theta3
            % Two values for theta3
            thetas(3,:) = 180 - psi - beta;
            
            % Two values for gamma, tau is a constant
            gamma = asind(L3*sind(beta)/dw);
            tau = asind(24*sind(psi)/128);

            % One value for alpha
            alpha = atan2d(zw,rw);
            
            % 90 = alpha + gamma + tau + theta2
            % Two values for theta2
            thetas(2,:) = 90 - tau - gamma - alpha;
            
            % phi = -theta2 - theta3 - theta4
            % Two values for theta4
            thetas(4,:) = -thetas(2,:) - thetas(3,:) - phi;

            % Now check if each row in thetas is a valid solution based on
            % the physical joint limits:
            % Joint 1: (-180 180) (None)
            % Joint 2: (-115 115)
            % Joint 3: (-115 85)
            % Joint 4: (-100 120)
            limits = [-180 180;-120 120;-120 90;-105 125];

            q = [];
            for i = 1:2
                valid = true;
                for j = 1:4
                    angle = thetas(j,i);
                    if angle < limits(j,1) || angle > limits(j,2)
                        valid = false;
                    end
                end
                if (valid)
                    q = [q; thetas(:,i)'];
                end
            end
        end
        
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            for i = 1:length(self.motors)
                motor = self.motors(i);
                readings(:,i) = motor.getJointReadings();
                % fprintf('[ID:%03d] PresPos:%.9f\tPresVel:%.3f\tPresCur:%.3f\n', motor.ID, readings(1,i), readings(2,i), readings(3,i));
            end
        end

        function setOperatingMode(self, mode)
            for i=1:length(self.motors)
                motor = self.motors(i);
                motor.setOperatingMode(mode);
            end
        end

        function writeJoints(self, goals)
            for i=1:length(self.motors)
                motor = self.motors(i);
                motor.writePosition(goals(i));
            end
        end
        

        function writeMotorState(self, enable)
            for i=1:length(self.motors)
                self.motors(i).toggleTorque(enable);
            end
        end

        function writeTime(self, time, acc_time)
            if ~exist("acc_time", "var")
                acc_time = time/3;
            end
            for i=1:length(self.motors)
                self.motors(i).writeTime(time, acc_time);
            end
        end

        function writeGripper(self, isOpen)
            gripper = self.gripper;
            if isOpen
                gripper.writePosition(-35);
            else
                gripper.writePosition(55);
            end
        end
    end
end