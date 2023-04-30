%% Defines the TrajPlanner class
% Calculates trajectories for different degrees and relevant coefficients
classdef TrajPlanner
    
    % Defines class variables
    properties
        % List of points to travel to
        setpoints;
    end

    % Defines class methods
    methods

        % Constructor
        function self = TrajPlanner(setpoints) 
            self.setpoints = setpoints;
        end

        %% Quintic Trajectory Methods

        % Given the time between setpoints, and number of points between
        % waypoints, returns the quintic trajectory
        % travelTime [int] - time between setPoints
        % pointsNum [int] - number of waypoints between setpoints
        function wayPointsList = getQuinticTraj(self, travelTime, pointsNum)
            setPoints = self.setpoints;
            wayPointsList = zeros((height(setPoints)-1)*(pointsNum+1)+1, 5);
            % Loops through each joint
            for i = 1:4
                count = 1;
                % Loops through each setPoint
                for j = 1:height(setPoints)-1
                    % Calculates coeffs for given setPoint for given joint
                    coeff = self.calcQuinticCoeff(0, travelTime, setPoints(j,i), setPoints(j+1,i), 0, 0, 0, 0)';
                    % Appends the starting setPoint to list
                    wayPointsList(count,2:end) = setPoints(j,:);
                    count = count + 1;
                    % Calculates and appends all wayPoints between current
                    % setPoint and next
                    wayPointsList(count:count+pointsNum-1,i+1) = self.calcQuinticTraj(travelTime, pointsNum, coeff);
                    count = count + pointsNum;
                end
                % Appends the final setPoint to list
                wayPointsList(count,2:end) = setPoints(height(setPoints),:);
            end
            % Calculates time for each point to travel
            time = linspace(0, travelTime*(height(setPoints)-1), height(wayPointsList))';
            % Adds the time to the list
            wayPointsList(:,1) = time;
        end
        
        % Given the time between setpoints, number of points between
        % waypoints, and polynomial coefficients, returns the quintic 
        % trajectory of wayPoints for a single pair of setPoints
        % travelTime [int] - time between setPoints
        % pointsNum [int] - number of waypoints between setpoints
        % coeff [N+1 doubles] - polynomial coefficients for trajectory
        function waypoints = calcQuinticTraj(self, travelTime, pointsNum, coeff)
            waypoints = zeros(pointsNum, 1);
            % Calculates difference in time for each wayPoint
            times = linspace(0,travelTime,pointsNum+2);
            % Loops through wayPoints between setPoints
            for k = 2:pointsNum+1
                % Extracts current relevant time from array
                t = times(k);
                % Calculates the wayPoint from polynomial coeffs and
                % appends to list
                waypoints(k-1,1) = coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5;
            end
        end

        % Given the initial time, final time, initial position, final
        % position, initial velocity, final velocity, initial acceleration,
        % and final acceleration, returns quintic polynomial coefficients
        % t0 [double] - start time of setPoint
        % tf [double] - end time of setPoint
        % p0 [double] - position of current setPoint
        % pf [double] - position of next setPoint
        % v0 [double] - starting velocity of setPoint
        % vf [double] - velocity at next setPoint
        % a0 [double] - starting acceleration of setPoint
        % af [double] - acceleration at next setPoint
        function coeff = calcQuinticCoeff(self, t0, tf, p0, pf, v0, vf, a0, af)
            % Known system of equations as matrix for a quintic trajectory
            coeffMatrix = [1 t0 t0^2 t0^3 t0^4 t0^5;...
                           0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; ...
                           0 0 2 6*t0 12*t0^2 20*t0^3; ...
                           1 tf tf^2 tf^3 tf^4 tf^5;...
                           0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; ...
                           0 0 2 6*tf 12*tf^2 20*tf^3];
            % Inputs to use for solving coefficients
            qs = [p0 v0 a0 pf vf af]';
            % Gets coefficients by division of matricies
            coeff = coeffMatrix\qs;
        end

        %% Cubic Trajectory Methods

        % Given the time between setpoints, and number of points between
        % waypoints, returns the cubic trajectory
        % travelTime [int] - time between setPoints
        % pointsNum [int] - number of waypoints between setpoints
        function wayPointsList = getCubicTraj(self, travelTime, pointsNum)
            setPoints = self.setpoints;
            wayPointsList = zeros((height(setPoints)-1)*(pointsNum+1)+1, 5);
            % Loops through each joint
            for i = 1:4
                count = 1;
                % Loops through each setPoint
                for j = 1:height(setPoints)-1
                    % Calculates coeffs for given setPoint for given joint
                    coeff = self.calcCubicCoeff(0, travelTime, setPoints(j,i), setPoints(j+1,i), 0, 0)';
                    % Appends the starting setPoint to list
                    wayPointsList(count,2:end) = setPoints(j,:);
                    count = count + 1;
                    % Calculates and appends all wayPoints between current
                    % setPoint and next
                    wayPointsList(count:count+pointsNum-1,i+1) = self.calcCubicTraj(travelTime, pointsNum, coeff);
                    count = count + pointsNum;
                end
                % Appends the final setPoint to list
                wayPointsList(count,2:end) = setPoints(height(setPoints),:);
            end
            % Calculates time for each point to travel
            time = linspace(0, travelTime*(height(setPoints)-1), height(wayPointsList))';
            % Adds the time to the list
            wayPointsList(:,1) = time;
        end

        % Given the time between setpoints, number of points between
        % waypoints, and polynomial coefficients, returns the cubic
        % trajectory of wayPoints for a single pair of setPoints
        % travelTime [int] - time between setPoints
        % pointsNum [int] - number of waypoints between setpoints
        % coeff [N+1 doubles] - polynomial coefficients for trajectory
        function waypoints = calcCubicTraj(self, travelTime, pointsNum, coeff)
            waypoints = zeros(pointsNum, 1);
            % Calculates difference in time for each wayPoint
            times = linspace(0,travelTime,pointsNum+2);
            % Loops through wayPoints between setPoints
            for k = 2:pointsNum+1
                % Extracts current relevant time from array
                t = times(k);
                % Calculates the wayPoint from polynomial coeffs and
                % appends to list
                waypoints(k-1,1) = coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3;
            end
        end

        % Given the initial time, final time, initial position, final
        % position, initial velocity, and final velocity, returns cubic 
        % polynomial coefficients
        % t0 [double] - start time of setPoint
        % tf [double] - end time of setPoint
        % p0 [double] - position of current setPoint
        % pf [double] - position of next setPoint
        % v0 [double] - starting velocity of setPoint
        % vf [double] - velocity at next setPoint
        function coeff = calcCubicCoeff(self, t0, tf, p0, pf, v0, vf)
            % Known system of equations as matrix for a cubic trajectory
            coeffMatrix = [1 t0 t0^2 t0^3 ;...
                           0 1 2*t0 3*t0^2 ; ...
                           1 tf tf^2 tf^3 ; ...
                           0 1 2*tf 3*tf^2];
            % Inputs to use for solving coefficients
            qs = [p0 v0 pf vf]';
            % Gets coefficients by division of matricies
            coeff = coeffMatrix\qs;
        end
    end
end