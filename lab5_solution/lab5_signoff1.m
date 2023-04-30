%% Configurations
clear
clc

% Creates camera object and performs calibrations
cam = Camera();
% Saves camera to mat file for later use so we do not have to calibrate the
% camera every time we want to use it
save("cam.mat");