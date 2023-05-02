close all
clear
clc

robot = Robot();
robot.writeMotorState(false);

plotFramesBool = true;
plotEEVelBool = true;
plotName = "ellipsoid model";
warning('off','MATLAB:fplot:NotVectorized')

figure('units','normalized','outerposition',[0 0 1 1])
while true
    plotArmRealtimeEllipsoid(plotFramesBool, plotEEVelBool, plotName)
    pause(0)
end


function plotArmRealtimeEllipsoid(plotFramesBool, plotEEVelBool, plotName)
    robot = Robot();
    robot.writeMotorState(false);

    read = robot.getJointsReadings();
    q = read(1,:);
    qDot = read(2,:);

    J = robot.getJacobian(q);
    pDot = J*qDot';
    % Give parameters default values, making them optional
    if ~exist("plotFramesBool", "var")
        plotFramesBool = true;
    end

    if ~exist("plotName", "var")
        plotName = "Robot Stick Model";
    end

    if ~exist("plotEEVelBool", "var")
        plotEEVelBool = true;
    end

    % Get coordinates of each joint using translation from T matrices
    tMats = robot.getAccMat(q);
    points = reshape(tMats(1:3,4,:),3,4);
    points = [[0;0;0] points]; % Add base frame origin

    % Plot Lines/Points
    p = plot3(points(1,:),points(2,:),points(3,:), '-o','Color','k','MarkerSize',10);
    hold on
    
    % Update existing plot
    set(p, 'XData', points(1,:), 'YData', points(2,:), 'ZData', points(3,:))
    
    if plotFramesBool % Plot coordinate arrows if prompted to
        colors = ['r','g','b'];
        arrowMag = 50;
        % Add coordinate frames to each point
        for i = 1:5
            for j = 1:3
                % Direction of current axis (j=1 for X, etc.) for base
                % frame
                dir = [(j==1) (j==2) (j==3)];

                if i ~= 1 % need to rotate according to T matrix
                    rot = tMats(1:3,1:3,i-1); % Take rotation part of T
                    dir = (rot * dir')';
                end

                % Calculate directions of the current axis
                u = double(dir(1));
                v = double(dir(2));
                w = double(dir(3));
                
                % Plot arrows and update existing plot
                h = quiver3(points(1,i),points(2,i),points(3,i), ...
                    dir(1),dir(2),dir(3), ... 
                    arrowMag, colors(j), 'LineWidth', 2);
                set(h, 'XData', points(1,i), 'YData', points(2,i), 'ZData', points(3,i), ...
                    'udata', u,'vdata', v,'wdata', w, ...
                    'MaxHeadSize',5e2);
            end
        end

        % Add legend for each arrow color
    end

    if plotEEVelBool

        eeLinVels = pDot(1:3);
        arrowMag = norm(eeLinVels);
    
        % Calculate directions of the current axis
        u = eeLinVels(1)/arrowMag;
        v = eeLinVels(2)/arrowMag;
        w = eeLinVels(3)/arrowMag;
    
        c = 'r';
        
        % Plot arrows and update existing plot
        h = quiver3(points(1,5),points(2,5),points(3,5), ...
            u,v,w, ... 
            arrowMag, c, 'LineWidth', 2);
        set(h, 'XData', points(1,5), 'YData', points(2,5), 'ZData', points(3,5), ...
            'udata', u,'vdata', v,'wdata', w, ...
            'MaxHeadSize',5e2);
    end

    titles = ["linear", "angular"];

    J = robot.getJacobian(q); 

    Jang = J(1:3,:);
    Jlin = J(4:6,:);
    for i = 1:1 %2
        curJ = J((i-1)*3+1:i*3,:);
        A = curJ*curJ'; 
        [V D] = eig(A);
        eigenvalues = diag(D);
        radii = sqrt(eigenvalues);
        volume = 4/3*pi*prod(radii);
        m1 = sqrt(max(eigenvalues)/min(eigenvalues));
        m2 = max(eigenvalues)/min(eigenvalues);
        m3 = sqrt(det(A));

        K = volume / m3; % constant: 4pi/3 for 3D ellipsoid

%         subplot(2,1,i)
        ellipsoidScale = 10;
        a = radii(1)*ellipsoidScale;
        b = radii(2)*ellipsoidScale;
        c = radii(3)*ellipsoidScale;

        funx = @(theta, phi) a*cos(theta).*cos(phi);
        funy = @(theta, phi) b*cos(theta).*sin(phi);
        funz = @(theta, phi) c*sin(theta);

        fun = @(theta, phi) [funx(theta, phi) funy(theta, phi) funz(theta, phi)]';

        funxPrime = @(theta, phi) V(:,1)'*fun(theta,phi) + points(1,end);
        funyPrime = @(theta, phi) V(:,2)'*fun(theta,phi) + points(2,end);
        funzPrime = @(theta, phi) V(:,3)'*fun(theta,phi) + points(3,end);
        
        funPrime = @(theta, phi) [funxPrime(theta, phi) ; funyPrime(theta, phi) ; funzPrime(theta, phi)];

        fs = fsurf(funxPrime,funyPrime,funzPrime,[-pi/2 pi/2 -pi pi]);
    end  

    hold off

    % Plot Formatting    
    title("ellipsoid model (volume: " + volume + ", manipulability: " + m3 + ", K: " + K + ")");
    legend('Link', 'X', 'Y', 'Z')
    grid on
    
    xlabel('x [mm]')
    ylabel('y [mm]')
    zlabel('z [mm]')

    xlim([-400 400]) 
    ylim([-400 400])
    zlim([0 500])
end