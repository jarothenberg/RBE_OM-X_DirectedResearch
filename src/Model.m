%% Defines the Model class
% Plots stick and dot model of the OpenManipulator-X model based on joint
% angles
classdef Model

    % Defines class variables
    properties
        % Robot Dimensions
        robot;
    end

    % Defines class methods
    methods

        % Constructor
        function self = Model() 
            self.robot = Robot();
        end

        %% Plotting Methods

        % Given q (the joint angles of the robot), plots a diagram of the
        % OpenManipulator-X arm in 3D
        % q [1x4 double] - the joint angles (rad) of the arm to be plotted
        % optional: plotFramesBool [logical] - whether to display the frames
        % on each joint of the arm.
        % optional: plotName [string] - Title for the plot to have.
        function plotArm(self, q, pDot, plotFramesBool, plotName)

            % Give parameters default values, making them optional
            if ~exist("plotFramesBool", "var")
                plotFramesBool = true;
            end

            if ~exist("plotName", "var")
                plotName = "Robot Stick Model";
            end

            % Get coordinates of each joint using translation from T matrices
            tMats = self.robot.getAccMat(q);
            points = reshape(tMats(1:3,4,:),3,4);
            points = [[0;0;0] points]; % Add base frame origin

            % Plot Lines/Points
            p = plot3(points(1,:),points(2,:),points(3,:), '-o','Color','k','MarkerSize',10);
            hold on
            
            % Update existing plot
            set(p, 'XData', points(1,:), 'YData', points(2,:), 'ZData', points(3,:))
            
            if plotFramesBool % Plot coordinate arrows if prompted to
                colors = ['r','g','b'];
                arrowMag = 50;
                % Add coordinate frames to each point
                for i = 1:5
                    for j = 1:3
                        % Direction of current axis (j=1 for X, etc.) for base
                        % frame
                        dir = [(j==1) (j==2) (j==3)];
    
                        if i ~= 1 % need to rotate according to T matrix
                            rot = tMats(1:3,1:3,i-1); % Take rotation part of T
                            dir = (rot * dir')';
                        end
    
                        % Calculate directions of the current axis
                        u = double(dir(1));
                        v = double(dir(2));
                        w = double(dir(3));
                        
                        % Plot arrows and update existing plot
                        h = quiver3(points(1,i),points(2,i),points(3,i), ...
                            dir(1),dir(2),dir(3), ... 
                            arrowMag, colors(j), 'LineWidth', 2);
                        set(h, 'XData', points(1,i), 'YData', points(2,i), 'ZData', points(3,i), ...
                            'udata', u,'vdata', v,'wdata', w, ...
                            'MaxHeadSize',5e2);
                    end
                end

                % Add legend for each arrow color
                legend('Link', 'X', 'Y', 'Z')
            end

            eeLinVels = pDot(1:3);
            arrowMag = norm(eeLinVels);

            % Calculate directions of the current axis
            u = eeLinVels(1)/arrowMag;
            v = eeLinVels(2)/arrowMag;
            w = eeLinVels(3)/arrowMag;

            c = 'r';
            
            % Plot arrows and update existing plot
            h = quiver3(points(1,5),points(2,5),points(3,5), ...
                u,v,w, ... 
                arrowMag, c, 'LineWidth', 2);
            set(h, 'XData', points(1,5), 'YData', points(2,5), 'ZData', points(3,5), ...
                'udata', u,'vdata', v,'wdata', w, ...
                'MaxHeadSize',5e2);

            hold off

            % Plot Formatting    
            title(plotName)

            grid on
            
            xlabel('x [mm]')
            ylabel('y [mm]')
            zlabel('z [mm]')

            xlim([-400 400]) 
            ylim([-400 400])
            zlim([0 500])
        end
        
        % Given some number of joint configurations and the number of
        % points to interpolate for each motion, simulate a motion between
        % all of the setpoint joint configurations using plotArm
        % qs - [nx4 double] - n setpoints to simulate motion for
        % sequentially
        % framesNum - int - Number of points to generate between each
        % setpoint to simulate motion in the 3D plot
        function dynamicPlot(self, qs, framesNum)
            preQ = qs(1,:);
            for k = 2:height(qs)
                q = qs(k,:);
                qLinspace = [linspace(preQ(1),q(1),framesNum); linspace(preQ(2),q(2),framesNum); linspace(preQ(3),q(3),framesNum); linspace(preQ(4),q(4),framesNum)]';
                preQ = q;
                for i = 1:framesNum
                    for j = 1:4 
                        qLinspace(i,j) = deg2rad(qLinspace(i,j));
                    end  
                end
    
                for i = 1:framesNum
                    plotArm(self, qLinspace(i,:), false);
                    pause(0.033);
                end
            end
        end

    end
end