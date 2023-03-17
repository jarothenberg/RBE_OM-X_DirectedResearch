%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2017 ROBOTIS CO., LTD.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Bulk Read and Bulk Write Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 2.0
% This example is designed for using two Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 1 (Baudrate : 57600)
%

clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

% Control table address
ADDR_PRO_TORQUE_ENABLE          = 64; %562;          % Control table address is different in Dynamixel model
ADDR_PRO_LED_RED                = 65; %563;
ADDR_PRO_GOAL_POSITION          = 116; %596;
ADDR_PRO_PRESENT_POSITION       = 132; %611;

% Data Byte Length
LEN_PRO_LED_RED                 = 1;
LEN_PRO_GOAL_POSITION           = 4;
LEN_PRO_PRESENT_POSITION        = 4;

% Protocol version
PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL1_ID                         = 11;            % Dynamixel#1 ID: 1
DXL2_ID                         = 12;            % Dynamixel#2 ID: 2
DXL3_ID                         = 13;            % Dynamixel#1 ID: 1
DXL4_ID                         = 14;            % Dynamixel#2 ID: 2
DXL5_ID                         = 15;            % Dynamixel#2 ID: 2
BAUDRATE                        = 1000000;
DEVICENAME                      = '/dev/ttyUSB0';       % Check which port is being used on your controller
                                                % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE      = 2048;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = 2048;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 20;           % Dynamixel moving status threshold

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port pathodd
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize groupBulkWrite Struct
groupwrite_num = groupBulkWrite(port_num, PROTOCOL_VERSION);

% Initialize Groupbulkread Structs
groupread_num = groupBulkRead(port_num, PROTOCOL_VERSION);

index = 1;
dxl_comm_result = COMM_TX_FAIL;                 % Communication result
dxl_addparam_result = false;                    % AddParam result
dxl_getdata_result = false;                     % GetParam result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                                  % Dynamixel error
dxl_led_value = [0 255];                        % Dynamixel LED value for write
dxl1_present_position = 0;                      % Present position
dxl2_led_value_read = 0;                        % Dynamixel moving status

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


% Disable Dynamixel#1 Torque
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel1 has been successfully connected \n');
end

% Disable Dynamixel#2 Torque
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel2 has been successfully connected \n');
end

% Disable Other Dynamixels
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

% Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL1_ID);
    return;
end

% Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL2_ID);
    return;
end

% Add parameter storage for Dynamixel#3 present position value
dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL3_ID);
    return;
end


% Add parameter storage for Dynamixel#4 present position value
dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL4_ID);
    return;
end

% Add parameter storage for Dynamixel#5 present position value
dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL5_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
if dxl_addparam_result ~= true
    fprintf('[ID:%03d] groupBulkRead addparam failed', DXL5_ID);
    return;
end

% % Add parameter storage for Dynamixel#3 present position value
% dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
% if dxl_addparam_result ~= true
%     fprintf('[ID:%03d] groupBulkRead addparam failed', DXL3_ID);
%     return;
% end
% 
% % Add parameter storage for Dynamixel#4 present moving value
% dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL4_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
% if dxl_addparam_result ~= true
%     fprintf('[ID:%03d] groupBulkRead addparam failed', DXL4_ID);
%     return;
% end
% 
% % Add parameter storage for Dynamixel#5 present moving value
% dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL5_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
% if dxl_addparam_result ~= true
%     fprintf('[ID:%03d] groupBulkRead addparam failed', DXL5_ID);
%     return;
% end

while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Add parameter storage for Dynamixel#1 goal position
%     dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, typecast(int32(dxl_goal_position(index)), 'uint32'), LEN_PRO_GOAL_POSITION);
%     if dxl_addparam_result ~= true
%       fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL1_ID);
%       return;
%     end

    % Add parameter storage for Dynamixel#1 LED value
    dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL1_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, dxl_led_value(index), LEN_PRO_LED_RED);
    if dxl_addparam_result ~= true
      fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL1_ID);
      return;
    end

    % Add parameter storage for Dynamixel#2 LED value
    dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL2_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, dxl_led_value(index), LEN_PRO_LED_RED);
    if dxl_addparam_result ~= true
      fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL2_ID);
      return;
    end

    % Add parameter storage for Dynamixel#3 LED value
    dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL3_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, dxl_led_value(index), LEN_PRO_LED_RED);
    if dxl_addparam_result ~= true
      fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL3_ID);
      return;
    end

    % Add parameter storage for Dynamixel#4 LED value
    dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL4_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, dxl_led_value(index), LEN_PRO_LED_RED);
    if dxl_addparam_result ~= true
      fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL4_ID);
      return;
    end

    % Add parameter storage for Dynamixel#5 LED value
    dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL5_ID, ADDR_PRO_LED_RED, LEN_PRO_LED_RED, dxl_led_value(index), LEN_PRO_LED_RED);
    if dxl_addparam_result ~= true
      fprintf(stderr, '[ID:%03d] groupBulkWrite addparam failed', DXL5_ID);
      return;
    end

    % Bulkwrite LED values
    groupBulkWriteTxPacket(groupwrite_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Clear bulkwrite parameter storage
    groupBulkWriteClearParam(groupwrite_num);

    while 1
        % Bulkread present position and moving status
        groupBulkReadTxRxPacket(groupread_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end

        % Check if groupbulkread data of Dynamixel#1 is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL1_ID);
            return;
        end

        % Check if groupbulkread data of Dynamixel#2 is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL2_ID);
            return;
        end

        % Check if groupbulkread data of Dynamixel#3 is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL3_ID);
            return;
        end

        % Check if groupbulkread data of Dynamixel#4 is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL4_ID);
            return;
        end

        % Check if groupbulkread data of Dynamixel#5 is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL5_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL5_ID);
            return;
        end

        % Get Dynamixels present position values
        dxl1_present_position = groupBulkReadGetData(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        dxl2_present_position = groupBulkReadGetData(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        dxl3_present_position = groupBulkReadGetData(groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        dxl4_present_position = groupBulkReadGetData(groupread_num, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        dxl5_present_position = groupBulkReadGetData(groupread_num, DXL5_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        
        % Convert to ints
        dxl1_present_position = typecast(uint32(dxl1_present_position), 'int32');
        dxl2_present_position = typecast(uint32(dxl2_present_position), 'int32');
        dxl3_present_position = typecast(uint32(dxl3_present_position), 'int32');
        dxl4_present_position = typecast(uint32(dxl4_present_position), 'int32');
        dxl5_present_position = typecast(uint32(dxl5_present_position), 'int32');

        fprintf('[ID:%03d] Pos: %d\n[ID:%03d] Pos: %d\n[ID:%03d] Pos: %d\n[ID:%03d] Pos: %d\n[ID:%03d] Pos: %d\n\n', ...
            DXL1_ID, dxl1_present_position, ...
            DXL2_ID, dxl2_present_position, ...
            DXL3_ID, dxl3_present_position, ...
            DXL4_ID, dxl4_present_position, ...
            DXL5_ID, dxl5_present_position);

%         if ~(abs(dxl_goal_position(index) - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD)
%             break;
%         end
    end

    % Change goal position
    if (index == 1)
      index = 2;        % Check if groupbulkread data of Dynamixel#4 is available
        dxl_getdata_result = groupBulkReadIsAvailable(groupread_num, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupBulkRead getdata failed', DXL4_ID);
            return;
        end
    else
      index = 1;
    end
end


% Disable Dynamixel#1 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Disable Dynamixel#2 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Disable Dynamixel#3 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Disable Dynamixel#4 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Disable Dynamixel#5 Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
