close all
clear
clc

robot = Robot();
robot.writeMotorState(false);

while true
    read = robot.getJointsReadings();
    q = read(1,:);
    J = robot.getJacobian(q);
    J = J(1:3,:);

    %% Based on Textbook
    [U D V] = svd(J);
    sigmas = diag(D);
    manipulability = prod(sigmas)

    %% Based on https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-4-manipulability/
    A = J*J'; 
    [V D] = eig(A);
    eigenvalues = diag(D);
    radii = sqrt(eigenvalues);
    volume = 4/3*pi*prod(radii);
    m1 = sqrt(max(eigenvalues)/min(eigenvalues));
    m2 = max(eigenvalues)/min(eigenvalues);
    m3 = sqrt(det(A))
end