% Skeleton Robot class for OpenManipulator-X Robot, for RBE 3001
% By Jack Rothenberg and Jatin Kohli

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350 classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (cm)
        mOtherDim; % Stores extraneous second link dimensions (cm)
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