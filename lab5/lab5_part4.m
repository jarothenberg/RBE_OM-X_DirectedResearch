%% Configurations
clear
clc

load("cam.mat") % Loads in camera
img = cam.getImage(); % Gets image from camera
imshow(img) % Shows image
% Arbitrary in world coordinates w.r.t. the robot
pointsRobot = [50 -100 ; 50 100 ; 125 -100 ; 125 100];
% Coresponding pixel coordinates of the same locations
pointsPixels = [444 314 ; 780 304 ; 401 400 ; 823 385];

% Initializes converted pixels to robot coords array to zeros
calcPointsRobot = zeros(height(pointsPixels), 3);
for i=1:height(pointsPixels) % Loops through all pixel coords
    % Calculates the pixel coordinates w.r.t. the robot frame, should be
    % negligibly close to chosen arbitrary points in robot space
    calcPointsRobot(i,:) = cam.pixelToRobot(pointsPixels(i,:));
end
% Displays the arbitrary robot points chosen
disp(pointsRobot)
% Displays the coresponding robot points calculated from pixel coords
disp(calcPointsRobot)