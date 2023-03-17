% Motor Class for the Dynamixel (X-Series) XM430-W350

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
        GROUPWRITE_NUM;
        GROUPREAD_NUM;

        % Control Table Constants - see link in constructor
        OPR_MODE;
        TORQUE_ENABLE;
        LED;
        GOAL_CURRENT;
        GOAL_VELOCITY;
        GOAL_POSITION;
        CURR_CURRENT;
        CURR_VELOCITY;
        CURR_POSITION;
        POS_LEN;
        VEL_LEN;
        CURR_LEN;

        % Variables
        ID; % Dynamixel ID
    end

    methods
        function self = DX_XM430_W350(id, deviceName)
            self.ID = id;
            self.PROTOCOL_VERSION = 2.0;
            self.DEVICENAME = deviceName;
            self.BAUDRATE = 1000000;
            self.COMM_SUCCESS = 0;
            self.COMM_TX_FAIL = -1001;

            % Load Libraries
            self.LIB_NAME = 'libdxl_x64_c';
            if ~libisloaded(self.LIB_NAME)
                [notfound, warnings] = loadlibrary(self.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
            end

            self.PORT_NUM = portHandler(self.DEVICENAME);

            % Initialize groupBulkRead/Write Struct
            self.GROUPWRITE_NUM = groupBulkWrite(self.PORT_NUM, self.PROTOCOL_VERSION);
            self.GROUPREAD_NUM = groupBulkRead(self.PORT_NUM, self.PROTOCOL_VERSION);

            % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table
            self.OPR_MODE = 11; % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
            self.TORQUE_ENABLE = 64; % Enable Torque Control
            self.LED = 65; % Turn LED on/off
            self.GOAL_CURRENT = 102;
            self.GOAL_VELOCITY = 104;
            self.GOAL_POSITION = 116; % Set/Get goal position
            self.CURR_CURRENT = 126;
            self.CURR_VELOCITY = 128;
            self.CURR_POSITION = 132; % Get current position
            self.POS_LEN = 4;
            self.VEL_LEN = 4;
            self.CURR_LEN = 2;

            self.startConnection();
        end

        function startConnection(self)
            packetHandler();

            % Open port
            if (openPort(self.PORT_NUM))
                fprintf('Succeeded to open the port!\n');
            else
                unloadlibrary(self.LIB_NAME);
                fprintf('Failed to open the port!\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Set port baudrate
            if (setBaudRate(self.PORT_NUM, self.BAUDRATE))
                fprintf('Succeeded to change the baudrate!\n');
            else
                unloadlibrary(self.LIB_NAME);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end
        end

        function stopConnection(self)
            closePort(self.PORT_NUM)
        end

        function readings = getJointReadings(self)  

            readings = zeros(1,3);

            cur_pos = read4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.CURR_POSITION);
            self.checkPacket();
            cur_vel = read4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.CURR_VELOCITY);
            self.checkPacket();
            cur_curr = read4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.CURR_CURRENT);
            self.checkPacket();
            disp(cur_pos)
            
            cur_vel = int32(cur_vel);
            cur_curr = int32(cur_curr);

            cur_pos = (cur_pos - 2048) * 360/4096;

            readings(1) = cur_pos;
            readings(2) = cur_vel;
            readings(3) = cur_curr;

%             fprintf('[ID:%03d] PresPos:%03d\tPresCur:%03d\tPresVel:%03d\n', self.ID, cur_pos, cur_curr, cur_vel);
        end

        function writePosition(self, angle)
            position = mod(typecast(uint32(angle * 4096/360 + 2048), 'int32'), 4096);
            write4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.GOAL_POSITION, position);
            self.checkPacket();
        end

        function writeVelocity(self, velocityInTicks)
            write4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.GOAL_VELOCITY, velocityInTicks);
            self.checkPacket();
        end

        function writeCurrent(self, currentInTicks)
            write4ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.GOAL_CURRENT, currentInTicks);
            self.checkPacket();
        end

        function toggleTorque(self, enable)
            % Enable Dynamixel#1 Torque
            write1ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.TORQUE_ENABLE, enable);
            self.checkPacket();
        end

        function toggleLED(self, enable)
            write1ByteTxRx(self.PORT_NUM, self.PROTOCOL_VERSION, self.ID, self.LED, enable);
            self.checkPacket();
        end

        function checkPacket(self)
            dxl_comm_result = getLastTxRxResult(self.PORT_NUM, self.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(self.PORT_NUM, self.PROTOCOL_VERSION);
            if dxl_comm_result ~= self.COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error));
            end
        end
    end
end