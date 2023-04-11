% Motor Class for the Dynamixel (X-Series) XM430-W350
% By Jack Rothenberg and Jatin Kohli
% The OpenManipulator-X arm consists of five of these (four joints + gripper)
% This class/file should not need to be modified in any way for RBE 3001, hopefully :)

classdef DX_XM430_W350
    properties
        % Constants
        DEVICENAME;
        LIB_NAME;
        BAUDRATE;
        PROTOCOL_VERSION;
        PORT_NUM
        COMM_SUCCESS;
        COMM_TX_FAIL;
        TRAV_TIME;

        % Control Table Constants - see link in constructor
        DRIVE_MODE;
        OPR_MODE;
        TORQUE_ENABLE;
        LED;
        GOAL_CURRENT;
        GOAL_VELOCITY;
        PROF_ACC;
        PROF_VEL;
        GOAL_POSITION;
        CURR_CURRENT;
        CURR_VELOCITY;
        CURR_POSITION;
        POS_LEN;
        VEL_LEN;
        CURR_LEN;
        CURR_CNTR_MD;
        VEL_CNTR_MD;
        POS_CNTR_MD;
        EXT_POS_CNTR_MD;
        CURR_POS_CNTR_MD;
        PWM_CNTR_MD;
        VEL_PROF;
        TIME_PROF;

        % Unit Conversions
        TICK_POS_OFFSET;
        TICKS_PER_ROT;
        TICKS_PER_DEG;
        TICKS_PER_ANGVEL;
        TICKS_PER_mA;
        MS_PER_S;

        % Variables
        ID; % Dynamixel ID
        % torqueEnable;
    end

    methods
        % Constructor to set up constants and connect via serial
        function self = DX_XM430_W350(id, deviceName)
            self.ID = id;
            self.PROTOCOL_VERSION = 2.0;
            self.DEVICENAME = deviceName;
            self.BAUDRATE = 1000000;
            self.COMM_SUCCESS = 0;
            self.COMM_TX_FAIL = -1001;

            % Load Libraries
            self.LIB_NAME = 'libdxl_x64_c'; % Linux 64
            if ~libisloaded(self.LIB_NAME)
                [notfound, warnings] = loadlibrary(self.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end
            self.PORT_NUM = portHandler(self.DEVICENAME);

            % Control Table Constants: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table
            self.DRIVE_MODE = 10; % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#drive-mode10
            self.OPR_MODE = 11; % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
            self.TORQUE_ENABLE = 64; % Enable Torque Control
            self.LED = 65; % Turn LED on/off
            self.GOAL_CURRENT = 102; % Set/Get goal current
            self.GOAL_VELOCITY = 104; % Set/Get goal velocity
            self.PROF_ACC = 108;
            self.PROF_VEL = 112;
            self.GOAL_POSITION = 116; % Set/Get goal position
            self.CURR_CURRENT = 126; % Get current current
            self.CURR_VELOCITY = 128; % Get current velocity
            self.CURR_POSITION = 132; % Get current position

            % message lengths in bytes
            self.POS_LEN = 4;
            self.VEL_LEN = 4;
            self.CURR_LEN = 2;

            % drive mode/operating mode constants
            self.CURR_CNTR_MD = 0;
            self.VEL_CNTR_MD = 1;
            self.POS_CNTR_MD = 3;
            self.EXT_POS_CNTR_MD = 4;
            self.CURR_POS_CNTR_MD = 5;
            self.PWM_CNTR_MD = 16;
            self.VEL_PROF = 0;
            self.TIME_PROF = 4;

            % Unit Conversions
            self.MS_PER_S = 1000;
            self.TICKS_PER_ROT = 4096;
            self.TICKS_PER_DEG = self.TICKS_PER_ROT/360;
            self.TICK_POS_OFFSET = self.TICKS_PER_ROT/2; % position value for a joint angle of 0 (2048 for this case)
            self.TICKS_PER_ANGVEL = 1/(0.229 * 6); % 1 tick = 0.229 rev/min = 0.229*360/60 deg/s
            self.TICKS_PER_mA = 1/2.69; % 1 tick = 2.69 mA

            self.startConnection();
        end

        % Attempts to connect via serial
        function startConnection(self)
            packetHandler();

            % Open port
            if (~openPort(self.PORT_NUM))
                unloadlibrary(self.LIB_NAME);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Set port baudrate
            if (~setBaudRate(self.PORT_NUM, self.BAUDRATE))
                unloadlibrary(self.LIB_NAME);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end
        end

        % Closes serial connection
        function stopConnection(self)
            closePort(self.PORT_NUM)
        end

        % Gets the joint readings (position, velocity, and current) of the motor
        % readings [1x3 double] - The joint positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointReadings(self)  
            readings = zeros(1,3);
            
            cur_pos = double(self.readWriteByte(self.POS_LEN, self.CURR_POSITION));
            cur_vel = double(self.readWriteByte(self.VEL_LEN, self.CURR_VELOCITY));
            cur_curr = double(self.readWriteByte(self.CURR_LEN, self.CURR_CURRENT));
            
            cur_pos = (cur_pos - self.TICK_POS_OFFSET) / self.TICKS_PER_DEG;
            cur_vel = cur_vel / self.TICKS_PER_ANGVEL;
            cur_curr = cur_curr / self.TICKS_PER_mA;
            
            readings(1) = cur_pos;
            readings(2) = cur_vel;
            readings(3) = cur_curr;
        end

        % Commands the motor to go to the desired angle
        % angle [double] - the angle in degrees for the motor to go to
        function writePosition(self, angle)
            position = mod(round(angle * self.TICKS_PER_DEG + self.TICK_POS_OFFSET), self.TICKS_PER_ROT);
            self.readWriteByte(self.POS_LEN, self.GOAL_POSITION, position);
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. 
        % time [double] - total profile time in s. If 0, the profile will be disabled.
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * self.MS_PER_S;
            acc_time_ms = acc_time * self.MS_PER_S;

            self.readWriteByte(4, self.PROF_ACC, acc_time_ms);
            self.readWriteByte(4, self.PROF_VEL, time_ms);
        end

        % Commands the motor to go at the desired angular velocity
        % velocity [double] - the angular velocity in deg/s for the motor to go at
        function writeVelocity(self, velocity)
            velTicks = velocity * self.TICKS_PER_ANGVEL; 
            self.readWriteByte(self.VEL_LEN, self.GOAL_VELOCITY, velTicks);
        end

        % Supplies the motor with the desired current
        % current [double] - the current in mA for the motor to be supplied with
        function writeCurrent(self, current)
            currentInTicks = current * self.TICKS_PER_mA;
            self.readWriteByte(self.CURR_LEN, self.GOAL_CURRENT, currentInTicks);
        end

        % Sets position holding on or off
        % enable [boolean] - true to enable torque to hold last set position, false to disable
        function toggleTorque(self, enable)
            self.readWriteByte(1, self.TORQUE_ENABLE, enable);
        end

        % Sets motor LED on or off
        % enable [boolean] - true to enable the LED, false to disable
        function toggleLED(self, enable)
            self.readWriteByte(1, self.LED, enable);
        end

        % Change the operating mode:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function setOperatingMode(self, mode)
            % if self.torqueEnable
            %     error("The motor torque must be disabled before running setOperatingMode.")
            % end

            % Save current motor state to go back to that when done
            currentMode = self.readWriteByte(1, self.TORQUE_ENABLE);

            self.toggleTorque(false);

            switch mode
                case {'current', 'c'} 
                    writeMode = self.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = self.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = self.POS_CNTR_MD;
                case {'ext position', 'ep'}
                    writeMode = self.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'}
                    writeMode = self.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'}
                    writeMode = self.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350 class.", mode)
            end

            self.readWriteByte(1, self.OPR_MODE, writeMode);
            self.toggleTorque(currentMode);
        end

        % Verifies if packet was sent/recieved properly, throws an error if not
        % addr [integer] - address of control table index of the last read/write
        % msg [integer] - the message (in bytes) sent with the last read/write 
        % msg is an optional parameter. Do not provide if the last serial communication was a read.
        function checkPacket(self, addr, msg)
            dxl_comm_result = getLastTxRxResult(self.PORT_NUM, self.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(self.PORT_NUM, self.PROTOCOL_VERSION);
            packetError = (dxl_comm_result ~= self.COMM_SUCCESS) || (dxl_error ~= 0);

            if exist("msg", "var") && packetError
                fprintf('[msg] %f\n', msg)
            end

            if dxl_comm_result ~= self.COMM_SUCCESS
                fprintf('[addr:%d] %s\n', addr, getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
                error("Communication Error: See above.")
            elseif dxl_error ~= 0
                fprintf('[addr:%d] %s\n', addr, getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
                error("Recieved Error Packet: See above.")
            end
        end

        % Reads or Writes the message of length n from/to the desired address
        % ignore the 2s complement stuff, we hadn't figured out typecast yet :)
        % n [integer] - The size in bytes of the message 
        % (1 for most settings, 2 for current, 4 for velocity/position)
        % addr [integer] - address of control table index to read from or write to
        % msg [integer] - the message (in bytes) to be sent with the write
        % msg is an optional parameter. Do not provide if trying to read from an address.
        function byte = readWriteByte(self, n, addr, msg)
            if exist("msg", "var") % write if msg variable was provided
                msg = round(msg);

                switch n % call method to write a certain number of bytes
                    case {1}
                        if msg < 0 % Convert to 2s complement 32 bit int
                            msg = 0xff + msg + 1;
                        end
                        msg = uint8(msg);
                        write1ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, addr, msg);
                    case {2}
                        if msg < 0 % Convert to 2s complement 32 bit int
                            msg = 0xffff + msg + 1;
                        end
                        msg = uint16(msg);
                        write2ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, addr, msg);
                    case {4}
                        
                        if msg < 0 % Convert to 2s complement 32 bit int
                            msg = 0xffffffff + msg + 1;
                        end
                        msg = uint32(msg);
                        write4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, addr, msg);
                    otherwise
                        error("'%s' is not a valid number of bytes to write.\n", n);
                end
                self.checkPacket(addr, msg);
            else % read if msg variable was not provided
                switch n % call method to read a certain number of bytes
                    case {1}
                        byte = read1ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, addr);
                        if byte > 0x7f % Convert to 2s complement 32 bit int
                            byte = 0xff - byte + 1;
                            byte = -1*double(byte);
                        end
                    case {2}
                        byte = read2ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, addr);
                        if byte > 0x7fff % Convert to 2s complement 32 bit int
                            byte = 0xffff - byte + 1;
                            byte = -1*double(byte);
                        end
                    case {4}
                        byte = read4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, addr);
                        if byte > 0x7fffffff % Convert to 2s complement 32 bit int
                            byte = 0xffffffff - byte + 1;
                            byte = -1*double(byte);
                        end
                    otherwise
                        error("'%s' is not a valid number of bytes to read.\n", n);
                end
                self.checkPacket(addr);
            end
        end
    end % end methods
end % end class