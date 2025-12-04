function [shorterX, shorterY] = compareDubinsPaths(fullX1, fullY1, fullX2, fullY2)

    % Calculate length of first path
    dx1 = diff(fullX1);
    dy1 = diff(fullY1);
    segmentLengths1 = sqrt(dx1.^2 + dy1.^2);
    length1 = sum(segmentLengths1);
    
    % Calculate length of second path
    dx2 = diff(fullX2);
    dy2 = diff(fullY2);
    segmentLengths2 = sqrt(dx2.^2 + dy2.^2);
    length2 = sum(segmentLengths2);

    if length1 < length2
        shorterX = fullX1; shorterY = fullY1;
    else
        shorterX = fullX2; shorterY = fullY2;
    end
end

function [fullX, fullY] = dubinsPathToPosition(start, goalPos, Rmin)
    
    n = 37;
    for k = linspace(0,360,n)
        goal = [goalPos, deg2rad(k)];

        ds_dub = 0.1;
        dubConn = dubinsConnection('MinTurningRadius', Rmin);
        cand_fullX = [];
        cand_fullY = [];
        cand_fullTheta = [];
        [segs, ~] = connect(dubConn, start, goal);
        if isempty(segs)
            warning('Dubins connection failed');
            return;
        end
        seg = segs{1};
        s = 0:ds_dub:seg.Length;
        poses = interpolate(seg, s);
        cand_fullX = [cand_fullX; poses(:,1)];
        cand_fullY = [cand_fullY; poses(:,2)];
        cand_fullTheta = [cand_fullTheta; poses(:,3)];

        if k == 0
            fullX = cand_fullX; 
            fullY = cand_fullY;
        else
            [fullX, fullY] = compareDubinsPaths(cand_fullX, cand_fullY, fullX, fullY);
        end
    end
end


% Define start: [x, y, theta] where theta is in radians
start = [0, 0, pi/4];  

% Define goal: [x, y]
goal = [-5, -5];

% Minimum turning radius
Rmin = 2;

% Compute path
[fullX, fullY] = dubinsPathToPosition(start, goal, Rmin);

% Plot the result
figure;

plot(fullX, fullY, 'b-', 'LineWidth', 2);
hold on;
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
axis equal;
grid on;

xlim([-10 10]);
ylim([-10 10]);
legend('Dubins Path', 'Start', 'Goal');