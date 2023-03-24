% Script to test IK math

clear all
clc

% Limits:
% Joint 1: (-180 180)
% Joint 2: (-115 115)
% Joint 3: (-115 85)
% Joint 4: (-100 120)

% Define link lengths
L1 = 77;
L2 = 130;
L3 = 124;
L4 = 126;

% Uncomment one block below to test
% 0 0 0 0
%%{
xe = 274;
ye = 0;
ze = 204.8;
phi = 0;
%}

% 45 -15 -60 30
%{
xe = 78.7032;
ye = 78.7032;
ze = 415.4938;
phi = -(-15 + (-60) + 30); % 45
%}

% -45 0 15 -45
%{  
xe = 178.8231;
ye = -178.8231;
ze = 235.6718;
phi = -(0 + 15 + (-45));
%}

% 105 -75 60 0
%{     
xe = -32.1663;
ye = 120.0464;
ze = 197.9551;
phi = -(-75 + 60 + 0);
%}

% Impossible
%{     
xe = 500;
ye = 500;
ze = 500;
phi = 0;
%}

theta1 = atan2d(ye,xe)

re = sqrt(xe^2 + ye^2);
rw = re - L4*cosd(phi);
zw = ze - L1 - L4*sind(phi);

dw = sqrt(rw^2 + zw^2);
alpha = atan2d(zw,rw);

%beta = acosd((L2^2 + L3^2 - dw^2)/(2*L2*L3));
cbeta = (L2^2 + L3^2 - dw^2)/(2*L2*L3);
sbeta = [sqrt(1-(cbeta)^2) -sqrt(1-(cbeta)^2)]
beta = [atan2d(sbeta(1),cbeta) atan2d(sbeta(2),cbeta)]

psi = atand(128/24);

% 180 = psi + beta + theta3
theta3 = 180 - psi - beta;

gamma = asind(L3*sind(beta)/dw);
tau = asind(24*sind(psi)/128);

% 90 = alpha + gamma + tau + theta2
theta2 = 90 - tau - gamma - alpha
theta3

% phi = -theta2 - theta3 - theta4
theta4 = -theta2 - theta3 - phi

