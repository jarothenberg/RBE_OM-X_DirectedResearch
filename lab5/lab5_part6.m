%% Configurations
clear
clc

% Loads camera object
load("cam.mat")

% List of example centroid pixels taken from image using data tips
centroidPixels = [414 356 ; 535 291 ; 630 314 ; 753 259];
for i=1:height(centroidPixels) % Loops through centroids
    % Gets the given centroid w.r.t. the robot
    cam.centroidToRobot(centroidPixels(i,:))
end