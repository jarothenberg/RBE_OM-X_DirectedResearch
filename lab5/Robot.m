%% Defines the Robot class
% Stores the state of the OpenManipulator-X
% Handles ROS communication
% Calculates serial manipulator operations
classdef Robot < handle

    % Defines class variables
    properties
        % ROS publishers
        mJointPosition_Pub; % Publishes the desired joint positions  
        mJointCurrent_Pub; % Publishes the desired joint currents
        mTrajectory_Pub; % Publishes the desired trajectory time
        mGripper_Pub; % Publishes the desired gripper state
        mMotorEnable_Pub; % Publishes the desired motor state
        mJointMode_Pub; % Publishes the joint mode
        % ROS subscribers
        mJointReadings_Sub; % Subscribets to the current joint readings
        % Class variables
        mJointReadings; % Stores the current joint readings
        mJointGoal; % Stores the current joint goal
        mDim; % Stores the robot link dimentions (cm)
        mOtherDim; % Stores extraneous second link dimensions (cm)
        % Forward Kinematics variables  
        mDHTable; % DH Table for the arm. Thetas need to be added to the joint angles first
    end

    % Defines class methods
    methods
 
        function self = Robot()
            % Initialize ROS publishers
            self.mJointPosition_Pub = rospublisher("/joint_goals", "open_manipulator_msgs/JointGoals", "DataFormat", "struct");
            self.mJointCurrent_Pub = rospublisher("/joint_currents", "open_manipulator_msgs/JointGoals", "DataFormat", "struct");
            self.mTrajectory_Pub = rospublisher("/trajectory_time", "std_msgs/Float32", "DataFormat", "struct");
            self.mGripper_Pub = rospublisher("/gripper_goal", "std_msgs/Bool", "DataFormat", "struct");
            self.mMotorEnable_Pub = rospublisher("/motor_enable", "std_msgs/Bool", "DataFormat", "struct");
            self.mJointMode_Pub = rospublisher("/joint_mode", "std_msgs/String", "DataFormat", "struct");
            % Initialize ROS subscribers
            self.mJointReadings_Sub = rossubscriber('/joint_readings','open_manipulator_msgs/JointReadings', @self.jointReadingsCallback);
            % Initialize class variables
            self.mJointReadings = repelem([struct('Position', 0, 'Velocity', 0, 'Effort', 0)], 4, 1); % (deg, deg/s, mA)
            self.mJointGoal = [0, 0, 0, 0]; % (deg)
            self.mDim = [77, 130, 124, 126]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            % Forward Kinematics variables
            self.mDHTable = [0 77 0 -90; (asind(24/130) - 90) 0 130 0; (90 - asind(24/130)) 0 124 0; 0 0 126 0];
        end

        %% Jacobian and Velocity

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

        % Returns the Jacobian matrix calculated using the analytic method.
        % This method is deprecated and not used in any code
        % q [1x4 double] - The joint angles of the robot arm in degrees
        % Returns: J [6x4 double] - Jacobian matrix
        function J = getJacobianAnalytic(self, q)
            % Sets angles from given q
            t1 = q(1);
            t2 = q(2);
            t3 = q(3);
            t4 = q(4);
            % Gets values from forward kinematics to be used in partial
            % derivatives
            tau = atan2d(24,128);
            re = 130*sind(t2+tau)+124*cosd(t2+t3)+126*cosd(t2+t3+t4);
            % Partial derivates of x over t1, t2, t3, t4
            x_t1 = -re*sind(t1);
            x_t2 = cosd(t1)*(130*cosd(t2+tau)-124*sind(t2+t3)-126*sind(t2+t3+t4));
            x_t3 = cosd(t1)*(-124*sind(t2+t3)-126*sind(t2+t3+t4));
            x_t4 = cosd(t1)*-126*sind(t2+t3+t4);
            % Partial derivates of y over t1, t2, t3, t4
            y_t1 = re*cosd(t1);
            y_t2 = sind(t1)*(130*cosd(t2+tau)-124*sind(t2+t3)-126*sind(t2+t3+t4));
            y_t3 = sind(t1)*(-124*sind(t2+t3)-126*sind(t2+t3+t4));
            y_t4 = sind(t1)*-126*sind(t2+t3+t4);
            % Partial derivates of z over t1, t2, t3, t4
            z_t1 = 0;
            z_t2 = -130*sind(t2+tau)-124*cosd(t2+t3)-126*cosd(t2+t3+t4);
            z_t3 = -124*cosd(t2+t3)-126*cosd(t2+t3+t4);
            z_t4 = -126*cosd(t2+t3+t4);
            % Partial derivates of phi over t1, t2, t3, t4
            phi_t1 = 0;
            phi_t2 = 0;
            phi_t3 = 0;
            phi_t4 = 0;
            % Partial derivates of theta over t1, t2, t3, t4
            theta_t1 = 0;
            theta_t2 = -1;
            theta_t3 = -1;
            theta_t4 = -1;
            % Partial derivates of psi over t1, t2, t3, t4
            psi_t1 = 1;
            psi_t2 = 0;
            psi_t3 = 0;
            psi_t4 = 0;
            % Assigns all to Jacobian matrix
            J = [x_t1, x_t2, x_t3, x_t4 ; ...
                 y_t1, y_t2, y_t3, y_t4 ; ...
                 z_t1, z_t2, z_t3, z_t4 ; ...
                 phi_t1, phi_t2, phi_t3, phi_t4 ; ...
                 theta_t1, theta_t2, theta_t3, theta_t4 ; ...
                 psi_t1, psi_t2, psi_t3, psi_t4];
        end

        %% Inverse Kinematics Methods

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

        % Return T0_4 based on the last read joint angles
        % (Transformation from end effector frame to base frame)
        % returns: 4x4 matrix: T0_4 using the last read joint angles
        function T = getCurrentFK(self)
            currentJoints = self.getJointsReadings();
            T = self.getFK(currentJoints(1,:));
        end

        % Return T0_4 based on the last set goal joint angles
        % (Transformation from end effector frame to base frame)
        % returns: 4x4 matrix: T0_4 using the last set goal joint angles
        function T = getGoalFK(self)
            T = self.getFK(self.mJointGoal);
        end

        % Return end-effector position (x,y,z in mm) based on the given
        % joint angles
        % returns: 1x3 array: Position wrt the base frame in mm in the
        % x,y,and z direction
        function eePos = getEEPos(self, q)
            T = self.getFK(q); % Get T matrices
            d = T(1:3,4)'; % Extract translation vector
            
            % TODO, extract roll pitch yaw from R

            eePos = [d -(q(2) + q(3) + q(4))]; % Transpose to get EE coordinates
        end

        %% ROS Subscribers

        % Reads and stores the joint states
        function jointReadingsCallback(self, src, msg)
            self.mJointReadings = msg.Readings;
        end

        %% ROS Publisher 
        
        % Writes the desired joint angles
        % goals [4x1 double] - The desired joint angles (deg)
        function writeJoints(self, goals)
            self.mJointGoal = goals;
            msg = rosmessage(self.mJointPosition_Pub);
            msg.Goals = goals;
            send(self.mJointPosition_Pub, msg);
        end

        % Writes the desired trajectory time
        % time [float] - The desired trajectory time (s)
        function writeTime(self, time)
            msg = rosmessage(self.mTrajectory_Pub);
            msg.Data = single(time);
            send(self.mTrajectory_Pub, msg);
            pause(0.5);
        end

        % Writes the desired gripper state
        % isOpen [bool] - The desired gripper state, true for open
        function writeGripper(self, isOpen)
            msg = rosmessage(self.mGripper_Pub);
            msg.Data = logical(isOpen);
            send(self.mGripper_Pub, msg);
        end

        % Writes the desired motor state
        % isEnabled [bool] - The desired motor state, true for enabled
        function writeMotorState(self, isEnabled)
            msg = rosmessage(self.mMotorEnable_Pub);
            msg.Data = isEnabled;
            send(self.mMotorEnable_Pub, msg);
            pause(0.1);
        end

        % Writes the desired current 
        % currents [4x1 double] - The desired currents (mA)
        function writeCurrents(self, currents)
            msg = rosmessage(self.mJointCurrent_Pub);
            msg.Goals = currents;
            send(self.mJointCurrent_Pub, msg);        
        end

        % Writes the desired mode
        % isPosition [bool] - The desire control mode, true for
        % position_mode
        function writeMode(self, isPosition)
            msg = rosmessage(self.mJointMode_Pub);
            if isPosition
                msg.Data = 'position_mode';
            else
                msg.Data = 'current_based_position_mode';
            end
            send(self.mJointMode_Pub, msg);
        end
        
        %% Basic Getters

        % Returns the joint positions, velocities, and currents
        % readings [3x4 double] - The joint positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = [];
            for reading = self.mJointReadings
                readings = [readings; reading.Position; reading.Velocity; reading.Effort];
            end
            pause(0.01); 
        end

        % Returns the joint positions, velocities, and currents
        % readings [3x4 double] - The joint positions, velocities,
        % and efforts (rad, rad/s, mA)
        function readings = getJointsReadingsRadians(self)
            readings = [];
            for reading = self.mJointReadings
                for i = 1:4
                    reading(i).Position = deg2rad(reading(i).Position);
                    reading(i).Velocity = deg2rad(reading(i).Velocity);
                end
                readings = [readings; reading.Position; reading.Velocity; reading.Effort];
            end
            pause(0.01); 
        end

        % Converts an array of angles from degrees to radians
        % anglesArray [1xn double] - The angles to convert
        function angles = degsToRads(self, anglesArray)
            angles = zeros(1, length(anglesArray));
            for i = 1:length(anglesArray)
                angles(i) = deg2rad(anglesArray(i));
            end
        end

        % Converts an array of angles from radians to degrees
        % anglesArray [1xn double] - The angles to convert
        function angles = radsToDegs(self, anglesArray)
            angles = zeros(1, length(anglesArray));
            for i = 1:length(anglesArray)
                angles(i) = rad2deg(anglesArray(i));
            end
        end

        % Returns the joint current readings
        % currents [1x4 double] - The joint currents (mA)
        function currents = getCurrentReadings(self)
            currents = [];
            readings = self.getJointsReadings();
            for i = 1: size(readings, 2)
                current = readings(3,i);
                currents = [currents; current];
            end
        end

        % Returns the most recent joint goals
        % goal [double] - The most recent joint goals (deg)
        function goal = getJointGoal(self)
            goal = self.mJointGoal;
        end
    end
end

