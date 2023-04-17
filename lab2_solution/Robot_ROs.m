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
            self.mDHTable = [0 77 0 -pi/2; (asin(24/130) - pi/2) 0 130 0; (pi/2 - asin(24/130)) 0 124 0; 0 0 126 0];
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
            A(1,:) = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta)];
            A(2,:) = [sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)];
            A(3,:) = [0 sin(alpha) cos(alpha) d];
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
            currentJoints = self.getJointsReadingsRadians();
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
            eePos = d; % Transpose to get EE coordinates
            % TODO, extract roll pitch yaw from R
            R = T(1:3, 1:3); % Extract rotation matrix
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
                msg.Data = 'current_mode';
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

        function angles = degsToRads(self, anglesArray)
            angles = zeros(1, length(anglesArray));
            for i = 1:length(anglesArray)
                angles(i) = deg2rad(anglesArray(i));
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
