classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;      % Camera Parameters
        cam;         % Webcam Object
        cam_pose;    % Camera Pose (transformation matrix)
        cam_IS;      % Camera Intrinsics
        cam_R;       % Camera Rotation Matrix
        cam_T;       % Camera Translation Vector
        imagePoints; % Image points from grid calibration
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            % make sure that the webcam can see the whole checkerboard by
            % running webcam(2).preview in the Command Window
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.calculateCameraPos();
        end

        function tForm = getTForm(self)
            tForm = rigid3d([ self.cam_R, zeros(3,1); self.cam_T, 1 ]);
        end

        function cam_pose = getCameraPose(self)
            cam_pose = self.cam_pose;
        end

        function cam_IS = getCameraInstrinsics(self)
            cam_IS = self.cam_IS;
        end

        function cam_R = getRotationMatrix(self)
            cam_R = self.cam_R;
        end

        function cam_T = getTranslationVector(self)
            cam_T = self.cam_T;
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        % Returns the bounding values for hue value and saturation by
        % calibrating colors placed in known areas on the checkerboard
        % Returns: hsvParams [6x6 double] - bounding values for hue, value,
        % and saturation for the red, orange, yellow, and green balls, and
        % the black and white squares
        % redOneSides [bool] - whether the hue from the red ball is on only
        % one side of the hsv spectrum
        function [hsvParams redOneSided] = hsvCalib(self)
            % Initializes hsvParams array to zeros
            hsvParams = zeros(6,6);
            % Gets the imagePoints from the grid
            ips = self.imagePoints;
            % Gets image from the camera
            img = self.getImage();
            % Initializes mask of all balls to 0
            maskSum = zeros(height(img),length(img));
            for i=1:4 % Loops through all 4 ball colors
                % Calculates the center of the given color circle in pixels
                colorCenter = ips(2*4*(i-1)+4,:);
                % Generates x and y coords of a circle parametrically
                theta = linspace(0,2*pi,10000);
                coordsX = 25*cos(theta)+colorCenter(1);
                coordsY = 25*sin(theta)+colorCenter(2);
                % Creates a circular mask using the coordinates calculated
                A = roipoly(img,coordsX,coordsY); 
                A = bwareafilt(A,1); 
                A = bwconvhull(A); 
                % Combines the current mask with the previous mask,
                % allowing for all balls to be included in one mask
                maskSum = maskSum | A;
            end
            % Displays instruction text
            disp("Line up balls within set zones, close figure when done.")
            % Creates a figure with number 1
            figure(1);
            while ishandle(1) % Checks if previous figure was closed
                % Gets current image from camera
                img = self.getImage();
                % Creates mask img by applying previous mask to image
                maskImg = immultiply(img,repmat(maskSum,[1 1 3]));
                % Shows the mask image, resulting in a live feed showing
                % the four circular cutouts allowing for aligning before
                % the calibration script continues
                imshow(maskImg)
            end
            % Converts the image to hsv
            img = rgb2hsv(img);
            for i=1:6 % Loops through each color of interest
%                 figure
                if i < 5 % If a colored ball not a grid square
                    % Calculates the center of the given color circle in pixels
                    colorCenter = ips(2*4*(i-1)+4,:);
                    % Generates x and y coords of a circle parametrically
                    theta = linspace(0,2*pi,1000);
                    coordsX = 25*cos(theta)+colorCenter(1);
                    coordsY = 25*sin(theta)+colorCenter(2);
                else % If a grid square
                    % Sets the corners of the grid and squeezes in a bit to
                    % ensure no borders are visible
                    tl = ips(i,:) + [5 5];
                    tr = ips(i+4,:) + [-5 5];
                    bl = ips(i+1,:) + [5 -5];   
                    br = ips(i+5,:) + [-5 -5];
                    coordsX = [tl(1) tr(1) br(1) bl(1)];
                    coordsY = [tl(2) tr(2) br(2) bl(2)];
                end
                % Creates a mask of the shape determined above
                A = roipoly(img,coordsX,coordsY);
                A = bwareafilt(A,1); 
                A = bwconvhull(A); 
                maskImg = immultiply(img,repmat(A,[1 1 3]));
                % Gets current hue from masked image
                currentHue = maskImg(:,:,1);
                % Only looks at hues within the roi
                currentHue = currentHue(currentHue~=0);
                % If color is not red, therefore the hue can simply be a 
                % bounded range
                if i~=1 
                    % Sets maximum hue to the maximum hue
                    maxHue = max(currentHue,[],'all');
                    % Sets the minimum hue to the minimum hue
                    minHue = min(currentHue,[],'all');
                else % If color is red, therefore fancy stuff must happen
                    % Makes a copy of the reds for use of the upper range
                    upperReds = currentHue;
                    % Only looks at reds in upper half of hue
                    upperReds = upperReds(upperReds > 0.5);
                    % Sets the maximum hue to the minimum of the upper
                    % reds
                    maxHue = min(upperReds,[],'all');
                    % Makes a copy of the reds for use of the lower range
                    lowerReds = currentHue;
                    % Only looks at reds in lower half of hue
                    lowerReds = lowerReds(lowerReds < 0.5);
                    % Sets the minimum hue to the maximum of the lower
                    % reds
                    minHue = max(lowerReds,[],'all');
                    % Sets value of redOneSided to false, meaning the reds
                    % are found on both sides of the hue
                    redOneSided = false;
                    % Checks if no value for the max or min was found,
                    % meaning red is only on one side of hsv
                    if isempty(maxHue) || isempty(minHue)
                        % Sets redOneSided variable to true
                        redOneSided = true;
                        % Sets maximum hue to the maximum hue
                        maxHue = max(currentHue,[],'all');
                        % Sets the minimum hue to the minimum hue
                        minHue = min(currentHue,[],'all');
                    end
                end
                % Gets current saturation from masked image
                currentSat = maskImg(:,:,2);
                % Only looks at saturations within the roi
                currentSat = currentSat(currentSat~=0);
                % Sets maximum saturation to the maximum saturation
                maxSat = max(currentSat,[],'all');
                % Sets minimum saturation to the minimum saturation
                minSat = min(currentSat,[],'all');
                % Gets current value from masked image
                currentVal = maskImg(:,:,3);
                % Only looks at values within the roi
                currentVal = currentVal(currentVal~=0);
                % Sets the maximum value to the maximum value
                maxVal = max(currentVal,[],'all');
                % Sets the minimum value to the minimum value
                minVal = min(currentVal,[],'all');
                % Sets the found hsv parameters to given color's row
                hsvParams(i,:) = [minHue maxHue minSat maxSat minVal maxVal];
%                 maskImg = insertMarker(maskImg, self.imagePoints(:,:,:), 'o', 'Color', 'white', 'Size', 10);
%                 imshow(hsv2rgb(maskImg));
            end
        end
        
        % Returns the coordinates of the centroid from pixels w.r.t. the
        % robot's reference frame
        % centroidPixels [1x2 int] - The pixel coordinates to convert
        % Returns: roboCoords [1x2 double] - XY coordinates of the centroid
        % converted into the robot's reference frame
        function roboCoords = centroidToRobot(self, centroidPixels)
            % Variable to account for the gripper to not quite reach the
            % ball, pushing it a bit further based on the offset calcd
            sagTolerance = 2;
            
            % XYZ of the camera to the origin of the world frame in mm
            camX = 100;
            camY = 290;
            camZ = 180;
            % Radius of the ball in mm
            ballRad = 10;
            
            % Calculates the centroid in world coordinates
            centroidWorld = pointsToWorld(self.cam_IS,self.cam_R,self.cam_T,centroidPixels);
            
            % Distance in x and y from the centroid to the camera in world
            % coordinates
            centroidCamX = camX - centroidWorld(1);
            centroidCamY = camY - centroidWorld(2);
            
            % Pythagorean theorum to find distance from camera to centroid
            centroidDist = sqrt((centroidCamX)^2+(centroidCamY)^2);
            % Calculates distance adjustment based on similar triangles
            xyAdjust = ballRad*centroidDist/camZ;
            % Angle of camera to centroid
            centroidCamAngle = atan2d(centroidCamX, centroidCamY);
            % X and Y components of the distance adjustment
            xAdjust = xyAdjust*sind(centroidCamAngle)*sagTolerance;
            yAdjust = xyAdjust*cosd(centroidCamAngle)*sagTolerance;
            % True x and y coordinates of ball in world coordinates
            worldBallX = centroidWorld(1)+xAdjust;
            worldBallY = centroidWorld(2)+yAdjust;
            % Calculates the centroid w.r.t. the robot for debugging
%             centroidRoboCoords = self.worldToRobot([centroidWorld(1) centroidWorld(2)])
            % Converts the ball coordinates from world to robot
            roboCoords = self.worldToRobot([worldBallX worldBallY]);
        end
        
        % Converts points from the world frame to the robot's frame
        % pointsWorld [1x2 double] - The world coordinates to convert
        % Returns: roboCoords [1x2 double] - XY coordinates of the point
        % converted into the robot's reference frame
        function roboCoords = worldToRobot(self, pointsWorld)    
            % Transformation matrix of the robot to the world origin
            robotToGrid = [0 1 0 50 ; ...
                           1 0 0 -100 ; ...
                           0 0 -1 0 ; ...
                           0 0 0 1];
            % Converts the points in the world frame to the robot frame.
            % Need to add a 0 for z in world coordinates and append a 1 to
            % allow for matrix multiplication
            robotCoord = robotToGrid*[pointsWorld 0 1]';
            % Extracts the x and y translation for the coordinates w.r.t.
            % the robot's reference frame
            roboCoords = robotCoord(1:2)';
        end

        % Converts points from pixels from the camera image to the robot
        % pointsPixel [1x2 int] - The pixel coordinates to convert
        % Returns: roboCoords [1x2 double] - XY coordinates of the point
        % converted into the robot's reference frame
        function roboCoords = pixelToRobot(self, pointsPixel)
            % Converts the pixel coordinates to world coordinates
            pointsWorld = pointsToWorld(self.cam_IS,self.cam_R,self.cam_T,pointsPixel);
            % Converts the world coordinates to robot coordinates
            roboCoords = self.worldToRobot(pointsWorld);
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = calculateCameraPos(self)  % DO NOT USE
            % calculateCameraPos Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [self.imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-2)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(self.imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(self.imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            
            pose = [   R,    t';
                    0, 0, 0, 1];
        end
    end
end
