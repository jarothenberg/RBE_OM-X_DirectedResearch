% Robot class for OpenManipulator-X Robot

classdef Robot

    properties
        % Constants
        LIB_NAME;
        PROTOCOL_VERSION;
        PORT_NUM;
        BAUDRATE;
        COMM_SUCCESS;
        COMM_TX_FAIL;

        % DX_XM430_W350 Servos
        motorsNum;
        motorIDs;
        gripperID;
        motors; % 4 Motors, joints 1-4
        gripper; % Gripper end-effector

        mDim; % Stores the robot link dimentions (cm)
        mOtherDim; % Stores extraneous second link dimensions (cm)
        % Forward Kinematics variables  
        mDHTable; % DH Table for the arm. Thetas need to be added to the joint angles first

        groupwrite_num;
        groupread_num;

        % Conversions
        TICKS_PER_ROT;
        TICKS_PER_DEG;
        TICK_POS_OFFSET;
        TICKS_PER_ANGVEL;
        TICKS_PER_mA;
        MS_PER_S;
    end

    methods
        function self = Robot()
            % Load Libraries
            self.LIB_NAME = 'libdxl_x64_c'; % Linux 64
            if ~libisloaded(self.LIB_NAME)
                [notfound, warnings] = loadlibrary(self.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end

            self.PROTOCOL_VERSION = 2.0;
            self.BAUDRATE = 1000000;
            self.COMM_SUCCESS = 0;
            self.COMM_TX_FAIL = -1001;
            self.motorsNum = 4;
            self.motorIDs = [11, 12, 13, 14];
            self.gripperID = 15;

            self.MS_PER_S = 1000;
            self.TICKS_PER_ROT = 4096;
            self.TICKS_PER_DEG = self.TICKS_PER_ROT/360;
            self.TICK_POS_OFFSET = self.TICKS_PER_ROT/2; % position value for a joint angle of 0 (2048 for this case)
            self.TICKS_PER_ANGVEL = 1/(0.229 * 6); % 1 tick = 0.229 rev/min = 0.229*360/60 deg/s
            self.TICKS_PER_mA = 1/2.69; % 1 tick = 2.69 mA
            
            % Find serial port and connect to it
            try
                devices = serialportlist();
                ttyDevs = devices(contains(devices,"/dev/ttyUSB"));
                deviceName = convertStringsToChars(ttyDevs(1));
            catch exception
                error("Failed to connect via serial, no devices found.")
            end
            self.PORT_NUM = portHandler(deviceName);

            self.groupwrite_num = groupBulkWrite(self.PORT_NUM, self.PROTOCOL_VERSION);
            self.groupread_num = groupBulkRead(self.PORT_NUM, self.PROTOCOL_VERSION);
           
            % Create array of motors
            for i=1:self.motorsNum
                self.motors = [self.motors; DX_XM430_W350(self.motorIDs(i), deviceName)];
            end

            % Create Gripper and set operating mode/torque
            self.gripper = DX_XM430_W350(self.gripperID, deviceName);
            self.gripper.setOperatingMode('p');
            self.gripper.toggleTorque(true);

            self.setOperatingMode('p');
            self.writeMotorState(true);

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
            J(5,:) = -J(5,:); % Changes the pitch due to our convention
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
            J(1:3,:) = J(1:3,:).*(pi/180);
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
        % deg
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
        % deg
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
            
            readings(1, :) = (self.bulkReadWrite(4, self.gripper.CURR_POSITION) - self.TICK_POS_OFFSET) ./ self.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(4, self.gripper.CURR_VELOCITY) ./ self.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(2, self.gripper.CURR_CURRENT) ./ self.TICKS_PER_mA;
        end

        % Returns the joint positions, velocities, and currents
        % readings [3x4 double] - The joint positions, velocities,
        % and efforts (rad, rad/s, mA)
        function readings = getJointsReadingsRadians(self)
            readings = self.getJointsReadings();
            readings(1:2,:) = readings(1:2,:) .* pi/180;
        end

        function setOperatingMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = self.gripper.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = self.gripper.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = self.gripper.POS_CNTR_MD;
                case {'ext position', 'ep'}
                    writeMode = self.gripper.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'}
                    writeMode = self.gripper.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'}
                    writeMode = self.gripper.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350 class.", mode)
            end
            self.writeMotorState(false);
            self.bulkReadWrite(1, self.gripper.OPR_MODE, writeMode);
            self.writeMotorState(true);
        end

        function writeJoints(self, goals)
            goals = mod(round(goals .* self.TICKS_PER_DEG + self.TICK_POS_OFFSET), self.TICKS_PER_ROT);
            
            self.bulkReadWrite(4, self.gripper.GOAL_POSITION, goals);
        end

        function writeVelocities(self, vels)
            vels = round(vels .* self.TICKS_PER_ANGVEL);

            self.bulkReadWrite(4, self.gripper.GOAL_VELOCITY, vels);
        end

        function writeCurrent(self, currents)
            currentInTicks = round(currents .* self.TICKS_PER_mA);
            self.bulkReadWrite(2, self.gripper.GOAL_CURRENT, currentInTicks);
        end

        function writeMotorState(self, enable)
            self.bulkReadWrite(1, self.gripper.TORQUE_ENABLE, enable);
        end

        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * self.MS_PER_S;
            acc_time_ms = acc_time * self.MS_PER_S;

            self.bulkReadWrite(4, self.gripper.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(4, self.gripper.PROF_VEL, time_ms);
        end

        function writeGripper(self, isOpen)
            gripper = self.gripper;
            if isOpen
                gripper.writePosition(-35);
            else
                gripper.writePosition(55);
            end
        end

        function result = bulkReadWrite(self, n, addr, msgs)
            if ~exist("msgs", "var") % Bulk Read
                groupBulkReadClearParam(self.groupread_num);
                
                result = zeros(1,4);
                for id = self.motorIDs
                    % Add parameter storage for Dynamixel
                    dxl_addparam_result = groupBulkReadAddParam(self.groupread_num, id, addr, n);
                    if dxl_addparam_result ~= true
                        fprintf('[ID:%03d ADDR:%d] groupBulkRead addparam failed\n', id, addr);
                        return;
                    end
                end

                % Bulkread present position and moving status
                groupBulkReadTxRxPacket(self.groupread_num);
                dxl_comm_result = getLastTxRxResult(self.PORT_NUM, self.PROTOCOL_VERSION);
                if dxl_comm_result ~= self.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                end

                for i = 1:length(self.motorIDs)
                    id = self.motorIDs(i);
                    % Check if groupbulkread data of Dynamixel#1 is available
                    dxl_getdata_result = groupBulkReadIsAvailable(self.groupread_num, id, addr, n);
                    if dxl_getdata_result ~= true
                        fprintf('[ID:%03d ADDR:%d] groupBulkRead getdata failed\n', id, addr);
                        return;
                    end
                    
                    % Get Dynamixels present position values
                    readBits = groupBulkReadGetData(self.groupread_num, id, addr, n);
                    % Printing
                    switch (n)
                        case {1}
                            result(i) = typecast(uint8(readBits), 'int8');
                        case {2}
                            result(i) = typecast(uint16(readBits), 'int16');
                        case {4}
                            result(i) = typecast(uint32(readBits), 'int32');
                        otherwise
                            error("'%s' is not a valid number of bytes to read.\n", n);
                    end
                end
                
            else % Bulk Write
                switch(n)
                    case {1}
                        msgs = typecast(int8(msgs), 'uint8');
                    case {2}
                        msgs = typecast(int16(msgs), 'uint16');
                    case {4}
                        msgs = typecast(int32(msgs), 'uint32');
                    otherwise
                        error("'%s' is not a valid number of bytes to write.\n", n);
                end

                if length(msgs) == 1
                    msgs = repelem(msgs, 4);
                end

                groupBulkWriteClearParam(self.groupwrite_num);
                for i = 1:length(self.motorIDs)
                    id = self.motorIDs(i);
                    dxl_addparam_result = groupBulkWriteAddParam(self.groupwrite_num, id, addr, n, msgs(i), n);
                    if dxl_addparam_result ~= true
                        fprintf('[ID:%03d ADDR:%d MSG:%d] groupBulkWrite addparam failed\n', id, addr, msgs(i));
                        return;
                    end
                end
                
                % Bulkwrite
                groupBulkWriteTxPacket(self.groupwrite_num);
                dxl_comm_result = getLastTxRxResult(self.PORT_NUM, self.PROTOCOL_VERSION);
                if dxl_comm_result ~= self.COMM_SUCCESS
                    fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                end
                
            end
        end
    end
end