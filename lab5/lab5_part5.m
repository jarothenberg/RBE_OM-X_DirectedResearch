%% Configurations
clear
clc

% Loads camera
load("cam.mat")

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
while true % Runs forever
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
    for i=1:4 % Loops through the 4 given colors
        % Initializes color mask to zero
        colorMask = zeros(height(img), width(img));
        % Creates booleans to check if pixels fall in calibrated range for
        % given color's saturation and value
        satBool = (sat > minSats(i)-tolerance & sat < maxSats(i)+tolerance);
        valBool = (val > minVals(i)-tolerance & val < maxVals(i)+tolerance);
        % Creates a 2x2 subplot at position i
%         subplot(2,2,i)
%         hold on
        % Assigns correct color to subplot
%         title(colors(i))
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
                                        MinimumBlobArea = 500, ExcludeBorderBlobs = true);
        % Saves the bounded areas, centroids, and boxes
        [areas, centroids, boxes] = step(blobAnalysis, logical(colorMask));
        [~, idx] = sort(areas, "Descend");
        % Get the two largest components.
        boxes = double(boxes(idx, :));
        
        % Reduce the size of the image for display.
        
        % Insert labels for the coins.
        img = insertObjectAnnotation(img, "rectangle", ...
            boxes, colors(i));
%         imshow(colorMask)
%         hold off
    end
    % Shows image in rgb of all balls with labels
    imshow(img);
end

