clear
clc
model = Model();

q0 = [0 0 0 0];
q1 = [-23.668 43.5938 -41.5723 88.3301];
q2 = [-15, 20, -20, 40];
qs = [q0;q1;q2];
model.dynamicPlot(qs, 100);
