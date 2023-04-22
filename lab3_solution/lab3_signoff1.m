% Code for signoff 1 to print the calculated FK/IK for known positions.

clear
clc
robot = Robot();

ikPoses = [274 0 204.8 0 ; ... % 0 0 0 0
           78.7032 78.7032 415.4938 45 ; ... % 45 -15 -60 30
           178.8231 -178.8231 235.6718 30]; % -45 0 15 -45

for i=1:3
    try
        ik = robot.getIK(ikPoses(i,:))
    catch
        error("End-Effector Pose Unreachable")
    end
    fk = robot.getFK(ik)
end

