%% Configurations
clear
clc

% Loads camera object
load("cam.mat")

% List of example centroid pixels taken from image using data tips
centroidPixels = [426 286 ; 770 263 ; 388 372 ; 816 337];

for i=1:height(centroidPixels) % Loops through centroids
    % Gets the given centroid w.r.t. the robot
    cam.centroidToRobot(centroidPixels(i,:))
end