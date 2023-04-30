% Skeleton Robot class for OpenManipulator-X Robot, for RBE 3001
% By Jack Rothenberg and Jatin Kohli

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350 classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        mDHTable; % Stores the DH table constants (mm), (deg)
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Robot Dimensions
            self.mDim = [77, 130, 124, 126]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.mDHTable = [0 self.mDim(1) 0 -90;...
                            (asind(self.mOtherDim(2)/self.mOtherDim(1)) - 90) 0 self.mDim(2) 0;...
                            (90 - asind(self.mOtherDim(2)/self.mOtherDim(1))) 0 self.mDim(3) 0;...
                            0 0 self.mDim(4) 0];
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
            read = self.getJointsReadings();
            q = read(1,:);
            T = self.getFK(q);
        end

        % Return end-effector position (x,y,z in mm) and orientation (pitch degrees) based on the given
        % joint angles
        % returns: 1x4 array: Position wrt the base frame in mm in the
        % x,y,and z direction and pitch in degrees
        function eePos = getEEPos(self, q)
            T = self.getFK(q); % Get T matrices
            d = T(1:3,4)'; % Extract translation vector

            eePos = [d -(q(2) + q(3) + q(4))]; % Transpose to get EE coordinates
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* self.TICKS_PER_DEG + self.TICK_POS_OFFSET), self.TICKS_PER_ROT);
            self.bulkReadWrite(self.gripper.POS_LEN, self.gripper.GOAL_POSITION, goals);
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * self.MS_PER_S;
            acc_time_ms = acc_time * self.MS_PER_S;

            self.bulkReadWrite(4, self.gripper.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(4, self.gripper.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(1, self.gripper.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* self.TICKS_PER_mA);
            self.bulkReadWrite(self.gripper.CURR_LEN, self.gripper.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = self.gripper.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = self.gripper.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = self.gripper.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = self.gripper.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = self.gripper.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = self.gripper.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350 class.", mode)
            end

            self.writeMotorState(false);
            self.bulkReadWrite(1, self.gripper.OPR_MODE, writeMode);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(self.gripper.POS_LEN, self.gripper.CURR_POSITION) - self.TICK_POS_OFFSET) ./ self.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(self.gripper.VEL_LEN, self.gripper.CURR_VELOCITY) ./ self.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(self.gripper.CURR_LEN, self.gripper.CURR_CURRENT) ./ self.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* self.TICKS_PER_ANGVEL);
            self.bulkReadWrite(self.gripper.VEL_LEN, self.gripper.GOAL_VELOCITY, vels);
        end
    end % end methods
end % end classrobot.writeGripper(true)