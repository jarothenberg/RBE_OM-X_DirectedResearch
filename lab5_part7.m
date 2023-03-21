%% Configurations
clear
clc

%% Setup robot
travelTime = 3; %0.5 % Defines the travel time
gripperTime = 0.5; % Time for gripper to open and close
pauseTime = travelTime;
robot = Robot(); % Creates robot object
robot.writeTime(travelTime); % Write travel time
robot.writeMotorState(true); % Write position mode

startAngle = [0 -20 0 0]; % Angle arm starts

robot.writeJoints(startAngle); % Sends arm to starting angle
pause(travelTime);

load("cam.mat") % Loads camera in

% Gets corner coordinates of relevant task space
tl = cam.imagePoints(1,:);
bl = cam.imagePoints(4,:);
tr = cam.imagePoints(33,:);
br = cam.imagePoints(36,:);

% Adds buffer to task space
tlExpanded = tl + [-10 -50];
blExpanded = bl + [-75 35];
trExpanded = tr + [10 -50];
brExpanded = br + [75 35];
corners = [blExpanded; tlExpanded ; trExpanded ; brExpanded];

load("hsvCalib.mat") % Loads hsv calibration data

% Sets max and mins for HSV values for masking
minHues = hsvParams(:,1);
maxHues = hsvParams(:,2);
minSats = hsvParams(:,3);
maxSats = hsvParams(:,4);
minVals = hsvParams(:,5);
maxVals = hsvParams(:,6);

% Creates a figure at full screen
figure('units','normalized','outerposition',[0 0 1 1])

% Initializes variable checking if done to false
done = false;
% Gets initial time
tic;
while ~done % Checks if done variable is set
    img = cam.getImage(); % Gets current image from camera
%     img = insertMarker(img, cam.imagePoints(:,:,:), 'o', 'Color', 'red', 'Size', 10);

    % Cropping
    % Creates polygon with coordinates found above and masks everything
    % outside of the polygon so we only have the relevant inside
    A = roipoly(img,corners(:,1)',corners(:,2)'); % x and y are the x and y coordinates of the vertices of the polygon 
    A = bwareafilt(A,1); 
    A = bwconvhull(A); 
    img = immultiply(img,repmat(A,[1 1 3])); 
    % Converts the image to hsv and records the hue, value, and saturation
    imHSV = rgb2hsv(img);
    hue = imHSV(:, :, 1);
    sat = imHSV(:,:,2);
    val = imHSV(:,:,3);

    tolerance = 0.05; % Tolerance of saturation and value
    hueTolerance = 0; % Tolerance of hue
    colors = ["red" "orange" "yellow" "green"];
    ballCount = 0; % Count of balls seen initialized
    targetBallCentroid = [-1 -1]; % Centroid of target ball initialized
    targetBallColor = "default"; % Color of target ball initialized
    
    for i=1:4 % Loops through the 4 given colors
        % Initializes color mask to zero
        colorMask = zeros(height(img), width(img));
        % Creates booleans to check if pixels fall in calibrated range for
        % given color's saturation and value
        satBool = (sat > minSats(i)-tolerance & sat < maxSats(i)+tolerance);
        valBool = (val > minVals(i)-tolerance & val < maxVals(i)+tolerance);

        % Checks if not red or the hues of red are on one side of hsv
        if i~=1 | redOneSided
            % If it is then do a similar bounding for hue as saturation and
            % value
            hueBool = (hue > minHues(i)-hueTolerance & hue < maxHues(i)+hueTolerance);
        else
            % If it is not, do alternative bounding to capture both sides
            % of hue
            hueBool = (hue < minHues(i)+hueTolerance | hue > maxHues(i)-hueTolerance);
        end
        % Set color mask so only pixels where the hue, value, and
        % saturation are all within range are targeted
        colorMask(hueBool & satBool & valBool) = 1;
        % Conducts blob analysis on masked image
        blobAnalysis = vision.BlobAnalysis(AreaOutputPort = true,...
                                        CentroidOutputPort = true,...
                                        BoundingBoxOutputPort = true,...
                                        MinimumBlobArea = 500, MaximumBlobArea = 5000, ExcludeBorderBlobs = true);
        % Saves the bounded areas, centroids, and boxes
        [areas, centroids, boxes] = step(blobAnalysis, logical(colorMask));
        
        % Loops through all found centroids of the given color
        for j=1:height(centroids)
            % If the ball is closer to the camera than the target
            if centroids(j,2) > targetBallCentroid(2)
                % Set the target ball to the closer one and record the
                % centroid and respective color
                targetBallCentroid = centroids(j,:);
                targetBallColor = colors(i);
            end
        end

        [~, idx] = sort(areas, "Descend");
        % Get the two largest components.
        boxes = double(boxes(idx, :));
        % Adds all balls found to ball count
        ballCount = ballCount + height(boxes);
        % Insert labels for the coins.
        img = insertObjectAnnotation(img, "rectangle", ...
            boxes, colors(i));
    end
    % Shows image in rgb of all balls with labels
    imshow(img);

    if ballCount == 0 % Checks if no balls found
        done = true; % Sets variable to done as we are complete
    else
        % Otherwise move the target ball to the given sorting area
        
        % Gets location of target ball w.r.t. the robot
        targetBallRobot = cam.centroidToRobot(targetBallCentroid);

        % Displays the target ball's location
        disp(targetBallRobot)
        % Open the gripper
        robot.writeGripper(true)
        % Moves robot to above the target ball
        robot.writeJoints(robot.getIK([targetBallRobot 100 -90]))
        pause(pauseTime)
        % Moves robot to the target ball
        robot.writeJoints(robot.getIK([targetBallRobot 30 -90]))
        pause(pauseTime)
        % Closes gripper, grabbing ball
        robot.writeGripper(false)
        pause(gripperTime)
        % Moves robot above the balls to avoid collision
        robot.writeJoints(robot.getIK([targetBallRobot 100 -90]))
        pause(pauseTime)
        % Picks the sorting location based on the color of the target ball
        % and moves the robot above that location
        switch targetBallColor
            case "red"
                disp('red')
                robot.writeJoints(robot.getIK([0 200 150 -45]));
            case "orange"
                disp('orange')
                robot.writeJoints(robot.getIK([0 -200 150 -45]));
            case "yellow"
                disp('yellow')
                robot.writeJoints(robot.getIK([150 -200 150 -45]));
            case "green"
                disp('green')
                robot.writeJoints(robot.getIK([150 200 150 -45]));
        end
        pause(pauseTime);
        % Opens the gripper, dropping the ball in the given sorting location
        robot.writeGripper(true);
    end
end
% Displays the final time
disp(toc);