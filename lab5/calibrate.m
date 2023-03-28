%% Configurations
clear
clc
close all

% Creates camera and performs camera calibration
cam = Camera();
% Saves camera object to mat file for later use
save("cam.mat");

% Performs hsv calibration
[hsvParams redOneSided] = cam.hsvCalib();
% Saves hsv calibration parameters for later use
save("hsvCalib.mat", "hsvParams", "redOneSided");