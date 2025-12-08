%%% ALL FUNCTIONS

%% Generate Simple Dubins Path

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

        ds_dub = 1;
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


%% Helper functions
function x = rand_val(a,b)
    x = a + (b-a)*rand;
end

function d = dist(x1, y1, x2, y2)
% pointDistance  Compute Euclidean distance between (x1,y1) and (x2,y2)
    d = sqrt( (x2 - x1)^2 + (y2 - y1)^2 );
end

%% Simulate LIMO movement
function [xn, yn, thetan] = dubins_step(x, y, theta, omega, v, dt)
% dubins_step  Propagate a Dubins car one step forward
%
% Inputs:
%   x, y, theta  - current pose
%   omega        - turning rate (rad/s)
%   v            - forward velocity
%   dt           - time step (s)
%
% Outputs:
%   xn, yn, thetan - next pose

    xn      = x + v*cos(theta)*dt;
    yn      = y + v*sin(theta)*dt;
    thetan  = theta + omega*dt;
end

function [xn, yn, thetan] = move_dumb_limo(x,y,theta,Rmin,Vmax, dt)
    v = rand_val(0,Vmax);
    omega = rand_val(-v/Rmin, v/Rmin);

    xn      = x + v*cos(theta)*dt;
    yn      = y + v*sin(theta)*dt;
    thetan  = theta + omega*dt;
end

function [v, omega] = bot_step(limoX, limoY, limoTheta, targetX, targetY, Vmax, Rmin, atGoal)

    % SENDLIMOSTEP Sends a single velocity command to move LIMO toward target
    %
    % Inputs:
    %   limoX, limoY, limoTheta - current LIMO pose
    %   targetX, targetY        - target position
    %   tcpObj                  - existing tcpclient object
    
    %% ----------------- CONFIG (based on physical parameters of LIMO and default values that we found were effective) -----------------
    omegaMax = Vmax/Rmin;    % max angular velocity (rad/s)
    Kp_linear = 1.0;   % proportional gain for linear velocity
    Kp_angular = 2.0;  % proportional gain for angular velocity
    
    %% ----------------- COMPUTE ERRORS -----------------
    dx = targetX - limoX;
    dy = targetY - limoY;
    distance = sqrt(dx^2 + dy^2);
    
    desiredTheta = atan2(dy, dx);
    angleError = wrapToPi(desiredTheta - limoTheta);
    
    %% ----------------- CHECK TOLERANCE -----------------
    if atGoal
        v = 0;
        omega = 0;
    else
        % Proportional control
        v = Kp_linear * distance;
        omega = Kp_angular * angleError;
        
        % Limit velocities
        v = max(min(v, Vmax), -Vmax);
        omega = max(min(omega, omegaMax), -omegaMax);
    end
end

%% COLLISION AVOIDANCE

%% PARAMETER DEFININITIONS

start = [-8, -8, 0];  % Define start: [x, y, theta] where theta is in radians
goal = [10, 7]; % Define goal: [x, y]
Rmin = 2; % Minimum turning radius
Vmax = 2; % Maximum velocity
commandInterval = 0.1; % Time per command
targetPointNum = 1;

% Compute path
[fullX, fullY] = dubinsPathToPosition(start, goal, Rmin);

%% INITIALIZE PLOT
figure;

plot(fullX, fullY, 'bo-', 'LineWidth', 2); %plotting the dubins path
hold on;
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % plot start point
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % plot end point
axis equal;
grid on;

% Initalize rectangle to represent Limo
L = 2; W = 1;
limoX = [-L/2, L/2, L/2, -L/2];
limoY = [-W/2, -W/2, W/2,  W/2];


hBot = patch(limoX + start(1), limoY + start(2), 'blue','DisplayName','LIMO');
hArrow = quiver(0,0,0,0,'LineWidth',2,'Color','r','MaxHeadSize',2, 'DisplayName','Direction');

% Initialize rectangles representing other Limo's

% Pre allocate structure
limo = struct;

limos = 12; % number of other limos

for i = 1:limos
    % start the limos at some random position
    limo_startX = randi([-8 8]);
    limo_startY = randi([-8 8]);
    limo_startTheta = randi([0 360]);

    % limoX = [-L/2, L/2, L/2, -L/2];
    % limoY = [-W/2, -W/2, W/2,  W/2];

    R = [cos(limo_startTheta) -sin(limo_startTheta); sin(limo_startTheta) cos(limo_startTheta)];
    rotated = R * [limoX  + limo_startX; limoY + limo_startY];

    % Draw square and store handle
    h = fill(rotated(1,:), rotated(2,:), 'y');
    hArrow = quiver(0,0,0,0,'LineWidth',2,'Color','r','MaxHeadSize',0.5, 'DisplayName','Direction');
    set(h, 'DisplayName', sprintf('limo_%d', i));
    set(hArrow, 'DisplayName', sprintf('arrow_%d', i));

    % Store handle with dynamic field name
    limo.(sprintf('limo_%d', i)) = h;
    limo.(sprintf('arrow_%d', i)) = hArrow;
end





xlim([-10 10]);
ylim([-10 10]);


%% REAL-TIME PLOT UPDATING FOR SIMULATION
atGoal = 0;
lookOutDist = 10;
while ~atGoal

    % Update the position of each dumb limo
    for j = 1:limos
        limoID = limo.(sprintf('limo_%d', j));
        arrowID = limo.(sprintf('arrow_%d', j));
        xs = get(limoID, 'XData');
        ys = get(limoID, 'YData');
        
        currentX = mean(xs);
        currentY = mean(ys);

        dx = xs(2) - xs(1);
        dy = ys(2) - ys(1);
     
        currentTheta = atan2(dy, dx);

        [newX, newY, newTheta] = move_dumb_limo(currentX, currentY, currentTheta, Rmin, Vmax, commandInterval);;

        R = [cos(newTheta) -sin(newTheta); sin(newTheta) cos(newTheta)];
        rotated = R * [limoX; limoY];

        set(limoID, 'XData', rotated(1,:) + newX, ...
                     'YData', rotated(2,:) + newY);

        % Arrow length (tweak as needed)
        arrowLength = 10;  
        
        % Arrow direction in world coordinates
        arrow_dx = arrowLength * cos(newTheta);
        arrow_dy = arrowLength * sin(newTheta);
        
        % Update arrow position and direction
        set(arrowID, 'XData', newX, ...
                    'YData', newY, ...
                    'UData', arrow_dx, ...
                    'VData', arrow_dy);

       % disp("X: " + currentX + " Y: " + currentY + " theta: " + currentTheta);

    end
    
    xs_bot = get(hBot, 'XData');
    ys_bot = get(hBot, 'YData');

    cx_bot = mean(xs_bot);
    cy_bot = mean(ys_bot);

    dx_bot = xs_bot(2) - xs_bot(1);
    dy_bot = ys_bot(2) - ys_bot(1);
 
    ct_bot = atan2(dy_bot, dx_bot);

   
    % Choose Target Point
    tol = 0.5;
    
    targetX = fullX(targetPointNum);
    targetY = fullY(targetPointNum);
    if dist(cx_bot, cy_bot, targetX, targetY) < tol
        targetPointNum = targetPointNum + 1;
    end
    if(targetPointNum >= length(fullX))
        atGoal = 1;
        targetPointNum = targetPointNum - 1;
    end
    if(targetPointNum + 10 >= length(fullX))
        lookOutX = fullX(targetPointNum: end);
        lookOutY = fullY(targetPointNum: end);
    else
        lookOutX = fullX(targetPointNum:targetPointNum + 10);
        lookOutY = fullY(targetPointNum:targetPointNum + 10);
    end
    plot(fullX, fullY, 'bo-', 'LineWidth', 2); %plotting the dubins path
    plot(lookOutX, lookOutY, 'ro-', 'LineWidth', 2); %plotting the dubins path

    [v_bot, omega_bot] = bot_step(cx_bot, cy_bot, ct_bot, targetX, targetY, Rmin, Vmax, 0);
    
    [newX_bot, newY_bot, newTheta_bot] = dubins_step(cx_bot, cy_bot, ct_bot, omega_bot, v_bot, commandInterval);


    R = [cos(newTheta_bot) -sin(newTheta_bot); sin(newTheta_bot) cos(newTheta_bot)];
    rotated_bot = R * [limoX; limoY];

    set(hBot, 'XData', rotated_bot(1,:) + newX_bot, ...
                 'YData', rotated_bot(2,:) + newY_bot);

    pause(commandInterval);
    legend('Dubins Path', 'Start', 'Goal');
end

disp("PATH COMPLETE!")