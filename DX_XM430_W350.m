% Motor Class for the Dynamixel (X-Series) XM430-W350

classdef DX_XM430_W350

    properties
        % Constants
        DEVICENAME;
        BAUDRATE;
        PROTOCOL_VERSION;
        COMM_SUCCESS;

        % Control Table Constants - see link in constructor
        TORQUE_ENABLE;
        GOAL_POSITION;
        CURR_POSITION;

        % Variables
        id; % Dynamixel ID
    end

    methods
        function self = DX_XM430_W350(id)
            self.id = id;

            self.PROTOCOL_VERSION = 2.0;
            self.DEVICENAME = '/dev/ttyUSB0';
            self.BAUDRATE = 1000000;
            self.port_num = portHandler(DEVICENAME);
            self.COMM_SUCCESS

            % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table
            self.OPR_MODE = 11; % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
            self.TORQUE_ENABLE = 64; % Enable Torque Control
            self.GOAL_POSITION = 116; % Set/Get goal position
            self.CURR_CURRENT = 126;
            self.CURR_VELOCITY = 128;
            self.CURR_POSITION = 132; % Get current position
        end

        function readings = getJointsReadings(self)
            DEVICENAME                  = '/dev/ttyUSB0';       
            PROTOCOL_VERSION            = 2.0;          
            DXL_ID = 11;
            port_num = portHandler(DEVICENAME);
            TORQUE_ENABLE          = 64;
            GOAL_POSITION          = 116;
            CURR_POSITION       = 132;
            MIN_POS_VAL  = 0; % Dynamixel will rotate between this value
            MAX_POS_VAL  = 1000; % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
            BAUDRATE                    = 1000000;
            COMM_SUCCESS                = 0;            % Communication Success result value
            index = 1;
            % Common Control Table Address and Data 
            ADDR_OPERATING_MODE         = 11;          
            OPERATING_MODE              = 3;            % value for operating mode for position control                                
            TORQUE_ENABLE               = 1;            % Value for enabling the torque
            TORQUE_DISABLE              = 0;            % Value for disabling the torque
            DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

            dxl_CURR_POSITION = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, CURR_POSITION);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
            end

            fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, dxl_goal_position(index), typecast(uint32(dxl_CURR_POSITION), 'int32'));

            if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_CURR_POSITION), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                break;
            end
        end

        function writePosition(self, position)

        end
    end
end