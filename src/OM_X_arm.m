% Class for OpenManipulator-X arm
% Used to abstract serial connection and read/write methods from Robot class
% This class/file should not need to be modified in any way for RBE 3001, hopefully :)
% By Jack Rothenberg and Jatin Kohli

classdef OM_X_arm

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

        % Serial Communication variables
        groupwrite_num;
        groupread_num;
        deviceName;

        % Conversions
        TICKS_PER_ROT;
        TICKS_PER_DEG;
        TICK_POS_OFFSET;
        TICKS_PER_ANGVEL;
        TICKS_PER_mA;
        MS_PER_S;
    end

    methods
        % Constructor to create constants and connect via serial
        % Fun fact, only the motor IDs are needed for bulk read/write,
        % so no instances of the DX_XM430_W350 class are required for the joints.
        % We still provide them (see self.motors) because we found them useful for
        % controlling the motors individually on occasion.
        function self = OM_X_arm()
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

            % conversions (see control table: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table-of-ram-area)
            self.MS_PER_S = 1000;
            self.TICKS_PER_ROT = 4096;
            self.TICKS_PER_DEG = self.TICKS_PER_ROT/360;
            self.TICK_POS_OFFSET = self.TICKS_PER_ROT/2; % position value for a joint angle of 0 (2048 for this case)
            self.TICKS_PER_ANGVEL = 1/(0.229 * 6); % 1 tick = 0.229 rev/min = 0.229*360/60 deg/s
            self.TICKS_PER_mA = 1/2.69; % 1 tick = 2.69 mA
            
            % Find serial port and connect to it
            try
                devices = serialportlist();
                self.deviceName = convertStringsToChars(devices(1));
            catch exception
                error("Failed to connect via serial, no devices found.")
            end
            self.PORT_NUM = portHandler(self.deviceName);

            self.groupwrite_num = groupBulkWrite(self.PORT_NUM, self.PROTOCOL_VERSION);
            self.groupread_num = groupBulkRead(self.PORT_NUM, self.PROTOCOL_VERSION);
           
            % Create array of motors
            for i=1:self.motorsNum
                self.motors = [self.motors; DX_XM430_W350(self.motorIDs(i), self.deviceName)];
            end

            % Create Gripper and set operating mode/torque
            self.gripper = DX_XM430_W350(self.gripperID, self.deviceName);
            self.gripper.setOperatingMode('p');
            self.gripper.toggleTorque(true);
        end

        % Reads or Writes the message of length n from/to the desired address for all joints
        % result [integer 1x4] - the result of a bulk read, empty if bulk write
        % n [integer] - The size in bytes of the message 
        % (1 for most settings, 2 for current, 4 for velocity/position)
        % addr [integer] - address of control table index to read from or write to
        % msgs [integer 1x4] - the messages (in bytes) to send to each joint, respectively
        % msgs is optional. Do not provide if trying to read.
        % msgs can also be an integer, where the same message will be sent to all four joints.
        function result = bulkReadWrite(self, n, addr, msgs)
            if ~exist("msgs", "var") % Bulk Read if msgs does not exist
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

                for i = 1:self.motorsNum
                    id = self.motorIDs(i);
                    % Check if groupbulkread data of each joint is available
                    dxl_getdata_result = groupBulkReadIsAvailable(self.groupread_num, id, addr, n);
                    if dxl_getdata_result ~= true
                        fprintf('[ID:%03d ADDR:%d] groupBulkRead getdata failed\n', id, addr);
                        return;
                    end
                    
                    % Get the data for the motor from the bulk read
                    readBits = groupBulkReadGetData(self.groupread_num, id, addr, n);

                    % Convert and store in return value
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
                
            else % Bulk Write if msgs exists
                % Convert to send
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

                % if the same message should go to all joints, make 3 more copies of it
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
    end % end methods
end % end class