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
        % Note that our robot class defaults to turning the motor state to on,
        % so this needs to be above in any script using Model to change this
        function self = Model() 
            self.robot = Robot();
        end

        %% Plotting Methods

        % Given q (the joint angles of the robot), plots a diagram of the
        % OpenManipulator-X arm in 3D
        % q [1x4 double] - the joint angles (rad) of the arm to be plotted
        function plotArm(self, q, plotFramesBool, plotName)
            if ~exist("plotFramesBool","var")
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
            hold off

            % Plot Formatting    
            title(plotName)

            grid on
            
            xlabel('x [mm]')
            ylabel('y [mm]')
            zlabel('z [mm]')

            xlim([-100 400]) 
            ylim([-400 400])
            zlim([0 500])
        end
    end
end