classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;     % Camera Parameters
        cam;        % Webcam Object
        cam_pose;   % Camera Pose (transformation matrix)
        cam_IS;     % Camera Intrinsics
        cam_R;      % Camera Rotation Matrix
        cam_T;      % Camera Translation Vector
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
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-2)*25, :);
            worldPointSize = size(self.params.WorldPoints);
            imagePointSize = size(imagePoints);
            fprintf("World Points is %d x %d\n", worldPointSize(1), worldPointSize(2));
            fprintf("Image Points is %d x %d\n", imagePointSize(1), imagePointSize(2));
            fprintf("The checkerboard is %d squares long x %d squares wide\n", boardSize(1), boardSize(2));

            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);

            self.cam_R = R;
            self.cam_T = t;
            
            pose = [   R,    t';
                    0, 0, 0, 1];
        end
    end
end