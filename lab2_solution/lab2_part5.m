clear
clc
model = Model();

qs = [0 0 0 0 ; 15 -45 -60 90 ; -90 15 45 -45];

for i=1:height(qs)
    figure
    q = qs(i,:);
    model.plotArm(q)
    set(gca, "FontSize", 50)
end