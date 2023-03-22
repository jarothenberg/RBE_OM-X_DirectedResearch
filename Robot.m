% Robot class for OpenManipulator-X Robot

classdef Robot

    properties
        % DX_XM430_W350 Servos
        motorsNum;
        motors; % 4 Motors, joints 1-4
        gripper; % Gripper end-effector

        mDim; % Stores the robot link dimentions (cm)
        mOtherDim; % Stores extraneous second link dimensions (cm)
        % Forward Kinematics variables  
        mDHTable; % DH Table for the arm. Thetas need to be added to the joint angles first
    end

    methods
        function self = Robot()
            self.motorsNum = 4;
            motorIDs = [11, 12, 13, 14];
            gripperID = 15;

            % Find serial port and connect to it
            try
                devices = serialportlist();
                deviceName = convertStringsToChars(devices(1));
            catch exception
                error("Failed to connect via serial, no devices found.")
            end
            
            % Create array of motors
            for i=1:self.motorsNum
                self.motors = [self.motors; DX_XM430_W350(motorIDs(i), deviceName)];
            end

            % Create Gripper and set operating mode/torque
            self.gripper = DX_XM430_W350(gripperID, deviceName);
            self.gripper.setOperatingMode('p');
            self.gripper.toggleTorque(true);

            % Robot Dimensions
            self.mDim = [77, 130, 124, 126]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            % Forward Kinematics variables
            self.mDHTable = [0 77 0 -90; (asind(24/130) - 90) 0 130 0; (90 - asind(24/130)) 0 124 0; 0 0 126 0];

        end

        % Returns the velocity for the end effector in x, y, z, roll, 
        % pitch, and yaw, by multiplying the jacobian calculated with given
        % joint angles and velocities
        % q [1x4 double] - The joint angles of the robot arm in degrees
        % qDot [1x4 double] - The joint velocities of the robot in deg/s
        % Returns: pDot [1x6 double] - End effector velocities
        function pDot = getForwardDiffKinematics(self, q, qDot)
            % Creates jacobian from given q values
            J = self.getJacobian(q);
            % Finds end effector velocities
            pDot = J*qDot';
        end

        % Returns the Jacobian matrix calculated using the geometric method
        % q [1x4 double] - The joint angles of the robot arm in degrees
        % Returns: J [6x4 double] - Jacobian matrix
        function J = getJacobian(self, q)
            % First get array of transformations of each joint to base
            Ts = self.getAccMat(q);
            % Initialize a 6x4 for Jacobian
            J = zeros(6,length(q));
            % Gets origin of end effector
            eeOrigin = Ts(1:3,4,end);
            % Creates an array of origins of each joint w.r.t. base frame
            origins = [[0 0 0]' Ts(1:3,4,1) Ts(1:3,4,2) Ts(1:3,4,3)];
            % Creates an array of z vectors from each joint
            angulars = [[0 0 1]' Ts(1:3,3,1) Ts(1:3,3,2) Ts(1:3,3,3)];
            % Loops through each joint for each column of J
            for i=1:length(q)
                % Assigns the linear component of the jacobian of the given
                % joint to the cross product of the current joint's z axis
                % and the difference of the end effector and current origin
                linear = (pi/180)*cross(angulars(:,i), eeOrigin-origins(:,i));
                % Assigns the linear component and angular component to J
                J(:,i) = [linear ; angulars(:,i)];
            end
        end

        % Given the 4 joint angles of each joint, return the 4 T matrices
        % (transform from the joint i to the base)
        % jointAngles [1x4 double] - The joint angles of the robot arm in
        % rad
        % returns: 4x4x4 matrix: [T0_1 T0_2 T0_3 T0_4]
        function TMat = getAccMat(self, jointAngles)
            TMat = zeros(4,4,4);

            % Get A matrices to multiply together
            AMat = self.getIntMat(jointAngles);
            
            % T_1^0 = A_1
            TMat(:,:,1) = AMat(:,:,1);
            % post-multiply A matrices to get each T_i^0 matrix
            for i = 2:4
                TMat(:,:,i) = TMat(:,:,i-1)*AMat(:,:,i);
            end
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

        %% Forward Kinematics Methods

        % Given a row from the DH table, return the corresponding A matrix 
        % (Homogeneous Transformation Matrix).
        % row [1x4 double] - one row of the DH table to get the A matrix
        function A = getDHRowMat(self, row)
            theta = row(1);
            d = row(2);
            a = row(3);
            alpha = row(4);

            % Fill out A matrix based on DH conventions
            A(1,:) = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta)];
            A(2,:) = [sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta)];
            A(3,:) = [0 sind(alpha) cosd(alpha) d];
            A(4,:) = [0 0 0 1];
        end

        % Given the 4 joint angles of each joint, return the 4 A matrices
        % jointAngles [1x4 double] - The joint angles of the robot arm in
        % rad
        % returns: 4x4x4 matrix: [A0 A1 A2 A3]
        function AMat = getIntMat(self, jointAngles)
            AMat = zeros(4,4,4);

            % copy DH table and fill in with actual joint angles
            dhTable = self.mDHTable;
            dhTable(:,1) = dhTable(:,1) + jointAngles'; 

            % Calculate each A matrix from corresponding row of DH table
            for i = 1:4
                AMat(:,:,i) = self.getDHRowMat(dhTable(i,:));
            end
        end

        % Given the 4 joint angles of each joint, return T0_4
        % (Transformation from end effector frame to base frame)
        % jointAngles [1x4 double] - The joint angles of the robot arm in
        % rad
        % returns: 4x4 matrix: T0_4
        function T = getFK(self, jointAngles)
            % Get A matrices to multiply together
            AMat = self.getIntMat(jointAngles);

            % T_1^0 = A_1
            T = AMat(:,:,1);
            % post-multiply all A matrices in order to get in T_4^0
            for i = 2:4
                T = T * AMat(:,:,i);
            end
        end

        % Return end-effector position (x,y,z in mm) based on the given
        % joint angles
        % returns: 1x3 array: Position wrt the base frame in mm in the
        % x,y,and z direction
        function eePos = getEEPos(self, q)
            T = self.getFK(q); % Get T matrices
            d = T(1:3,4)'; % Extract translation vector

            eePos = [d -(q(2)+q(3)+q(4))]; % Transpose to get EE coordinates
        end
        
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            for i = 1:self.motorsNum
                motor = self.motors(i);
                readings(:,i) = motor.getJointReadings();
                % fprintf('[ID:%03d] PresPos:%.9f\tPresVel:%.3f\tPresCur:%.3f\n', motor.ID, readings(1,i), readings(2,i), readings(3,i));
            end
        end

        function setOperatingMode(self, mode)
            for i=1:self.motorsNum
                motor = self.motors(i);
                motor.setOperatingMode(mode);
            end
        end

        function writeJoints(self, goals)
            for i=1:self.motorsNum
                motor = self.motors(i);
                motor.writePosition(goals(i));
            end
        end

        function writeVelocities(self, vels)
            for i=1:self.motorsNum
                motor = self.motors(i);
                motor.writeVelocity(vels(i));
            end
        end
        

        function writeMotorState(self, enable)
            for i=1:self.motorsNum
                self.motors(i).toggleTorque(enable);
            end
        end

        function writeTime(self, time, acc_time)
            if ~exist("acc_time", "var")
                acc_time = time/3;
            end
            for i=1:self.motorsNum
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