%%% ALL FUNCTIONS

%% Generate Simple Dubins Path

% Compares two dubins path and returns the shorter one
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

% creates dubins path
function [fullX, fullY] = dubinsPathToPosition(start, goalPos, Rmin)

    % Inputs:
    %   start = [x,y, theta] of starting position
    %   goalPos = [x,y] of ending position

    n = 37; % number of angles checked;
    for k = linspace(0,360,n)
        goal = [goalPos, deg2rad(k)];

        ds_dub = 0.3;
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

% returns a random value between a and b
function x = rand_val(a,b)
    x = a + (b-a)*rand;
end

% pointDistance  Compute Euclidean distance between (x1,y1) and (x2,y2)
function d = dist(x1, y1, x2, y2)
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


function [xn, yn, thetan] = move_dumb_limo_sim(x,y,theta,Rmin,Vmax, dt)
    % controls limos to move randomly
    v = rand_val(0,Vmax);
    omega = rand_val(-v/Rmin, v/Rmin);

    xn      = x + v*cos(theta)*dt;
    yn      = y + v*sin(theta)*dt;
    thetan  = theta + omega*dt;
end

function [v, omega] = bot_step_sim(limoX, limoY, limoTheta, targetX, targetY, Vmax, Rmin, atGoal)

    % bot_step_sim returns the v and omega the bot requires to move towards
    % target point
    %
    % Inputs:
    %   limoX, limoY, limoTheta - current LIMO pose
    %   targetX, targetY        - target position
    
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
    
    %% ----------------- CHECK IF AT GOAL -----------------
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
function intersects = quiverIntersectsLine(hQuiver, lineX, lineY)
% quiverIntersectsLine  Check if a quiver intersects a piecewise line
%
% Inputs:
%   hQuiver - handle to quiver object
%   lineX   - vector of x coordinates of piecewise line
%   lineY   - vector of y coordinates of piecewise line
%
% Output:
%   intersects - true if the quiver intersects any segment of the line

    % Get quiver data
    x0 = hQuiver.XData;
    y0 = hQuiver.YData;
    u  = hQuiver.UData;
    v  = hQuiver.VData;

    % Quiver endpoint
    x1 = x0 + u;
    y1 = y0 + v;

    intersects = false;

    % Check each segment of the piecewise line
    for i = 1:length(lineX)-1
        % segment endpoints
        x2 = lineX(i);
        y2 = lineY(i);
        x3 = lineX(i+1);
        y3 = lineY(i+1);

        if segmentsIntersect(x0,y0,x1,y1,x2,y2,x3,y3)
            intersects = true;
            return
        end
    end
end

% -----------------------
function flag = segmentsIntersect(x1,y1,x2,y2,x3,y3,x4,y4)
% Check if line segment (x1,y1)-(x2,y2) intersects (x3,y3)-(x4,y4)
    
    % Compute denominators
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if den == 0
        flag = false; % parallel
        return
    end

    % Compute intersection point t and u
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den;
    u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / den;

    % Check if intersection is within both segments
    flag = (t >= 0 && t <= 1 && u >= 0 && u <= 1);
end

%% ----------------- HELPER FUNCTION FOR SENDING COMMANDS TO LIMO -----------------
function sendCmd(tcpipClient, v,w)
    try
        fwrite(tcpipClient, sprintf('%.4f,%.4f', v, w));
        fprintf("SEND: %.3f, %.3f\n", v, w);
    catch
        warning('TCP write failed.');
    end
end
%% MQTT SETUP
broker = "mqtt://rasticvm.lan"; 
clientID = "Winnie";

robotID = "#"; 
topic = "rb/" + robotID;
RobotPositions = containers.Map('KeyType', 'char', 'ValueType', 'any');

% Close previous client if it exists
try 
    clear mqttClient 
catch 
end

% Connect to broker 
try 
    mqttClient = mqttclient(broker, clientId=clientID); 
    fprintf("Connected to MQTT Broker: %s\n", broker); 
catch ME 
    error("MQTT:ConnectionFailed", ... 
        "Failed to connect to MQTT broker. Error: %s", ME.message);
end


% Subscribe
subscribe(mqttClient, topic); 
fprintf("Subscribed to topic: %s\n", topic);

%% TCP SETUP
limoAddress = '192.168.1.172';   % Set this to the Limo's IP address
limoPort    = 12345;             % Do not change

runLimo = true;

if runLimo
    tcpipClient = tcpclient(limoAddress, limoPort);
    % fopen(tcpipClient);
    disp('Connected to Limo.');
else
    tcpipClient = [];
end


%% PARAMETER DEFININITIONS (CONFIG)

botID = "limo809";
botTopic = "rb/" + botID;

start = [0, 0, pi];  % Define start: [x, y, theta] where theta is in radians
goal = [2, 3]; % Define goal: [x, y]
Rmin = 1; % Minimum turning radius
Vmax = 3; % Maximum velocity
commandInterval = 0.1; % Time per command


limos = 0; % number of other limos

lookOutDist = 2; % how many checkpoints ahead the limo looks to see if anything is in it's path
tol = 0.1; % how close to point to be considered 'at' the point
recalDist = 0.5; % how far from goal point to recalibrate path
arrowLength = 0.75;



%% INITIALIZE PLOT

% find start

msg = read(mqttClient);

if ~isempty(msg)
    for k = 1:height(msg)
         t    = msg.Topic{k};
        data = jsondecode(msg.Data{k});

        % ----------- Robust Position Extraction (x, z) -----------
        pos = struct('x', [], 'z', []);
        if isfield(data, "pos")
            p = data.pos;

            % Case A: {"pos":{"x":..,"y":..,"z":..}}
            if isstruct(p)
                if isfield(p, "x"), pos.x = p.x; end
                if isfield(p, "z"), pos.z = p.z; else, pos.z = 0; end

            % Case B: {"pos":[x,y,z]}
            elseif isnumeric(p)
                if numel(p) >= 1, pos.x = p(1); end
                if numel(p) >= 3, pos.z = p(3); else, pos.z = 0; end
            end

        % Case C: {"x":.., "z":..}
        elseif isfield(data, "x")
            pos.x = data.x;
            if isfield(data, "z"), pos.z = data.z; else, pos.z = 0; end
        end
        % ----------------------------------------------------------
        
        % Skip invalid messages
        if isempty(pos.x)
            fprintf("Invalid position from %s\n", t);
            continue
        end

        robotX = - pos.x;
        robotY = pos.z;

        qx = data.rot(1);
        qy = data.rot(2);
        qz = data.rot(3);
        qw = data.rot(4);
        q = [qw, qx, qy, qz];

        ROT = quat2rotm(q);
        robotTheta = pi - atan2(ROT(3,1), ROT(1,1));   % full -180° to +180° -> NOTE we cannot use roll-pitch-yaw because pitch is limited to 180° range

        % Store latest robot position
        RobotPositions(t) = pos;

        % --- Update or create plot element ---
        if (t == botTopic)
            % Update my bot's position (REQUIRES IMPLEMENTATION)
            start = [robotX, robotY, robotTheta];
        end
    end
end


% Compute path
[fullX, fullY] = dubinsPathToPosition(start, goal, Rmin);

figure;

% plot(fullX, fullY, 'bo-', 'LineWidth', 2); %plotting the dubins path
hold on;
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % plot start point
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % plot end point
axis equal;
grid on;

% Initalize rectangle to represent Limos; L = limo length, W = limo width
L = 0.3; W = 0.2;
limoX = [-L/2, L/2, L/2, -L/2];
limoY = [-W/2, -W/2, W/2,  W/2];

% Initialize our 'bot' which is the LIMO we control
hBot = patch(limoX + start(1), limoY + start(2), 'blue','DisplayName','LIMO');


%% INITIALIZE ALL OTHER LIMOS
% Pre allocate structure
limo = struct;

% Choose Graph Size (change to fit to match RASTIC's mocap area)
xlim([-5 5]);
ylim([-5 5]);

robotPlots = containers.Map;

% Make sure path exists
if isempty(fullX) || isempty(fullY)
    error('Path generation failed: fullX/fullY empty. Check dubinsPathToPosition output.');
end

% Plot once and keep handles for updates
hPath = plot(fullX, fullY, 'bo-', 'LineWidth', 2); % full planned path
hold on;
hLookOut = plot(NaN, NaN, 'ro-', 'LineWidth', 2);  % lookout segment (updated below)

%% REAL-TIME PLOT UPDATING FOR SIMULATION (MAIN LOOP)

% Pre-loop initializations
atGoal = 0; % atGoal is 0 if bot not at goal, 1 if it is
intersection = 0; % initialize boolean checking fi there is a limo in the immediate path
targetPointNum = 1; % initalize which poin the Limo is trying to go
            
while ~atGoal
    

    % Update the position of each dumb limo
    intersection = 0;
    % Read all new messages
    msg = read(mqttClient);

    if ~isempty(msg)
        for k = 1:height(msg)
            t    = msg.Topic{k};
            data = jsondecode(msg.Data{k});

            % ----------- Robust Position Extraction (x, z) -----------
            pos = struct('x', [], 'z', []);
            if isfield(data, "pos")
                p = data.pos;

                % Case A: {"pos":{"x":..,"y":..,"z":..}}
                if isstruct(p)
                    if isfield(p, "x"), pos.x = p.x; end
                    if isfield(p, "z"), pos.z = p.z; else, pos.z = 0; end

                % Case B: {"pos":[x,y,z]}
                elseif isnumeric(p)
                    if numel(p) >= 1, pos.x = p(1); end
                    if numel(p) >= 3, pos.z = p(3); else, pos.z = 0; end
                end

            % Case C: {"x":.., "z":..}
            elseif isfield(data, "x")
                pos.x = data.x;
                if isfield(data, "z"), pos.z = data.z; else, pos.z = 0; end
            end
            % ----------------------------------------------------------
            
            % Skip invalid messages
            if isempty(pos.x)
                fprintf("Invalid position from %s\n", t);
                continue
            end

            robotX = - pos.x;
            robotY = pos.z;

            qx = data.rot(1);
            qy = data.rot(2);
            qz = data.rot(3);
            qw = data.rot(4);
            q = [qw, qx, qy, qz];
    
            ROT = quat2rotm(q);
            robotTheta = pi - atan2(ROT(3,1), ROT(1,1));   % full -180° to +180° -> NOTE we cannot use roll-pitch-yaw because pitch is limited to 180° range

            % Store latest robot position
            RobotPositions(t) = pos;

            % --- Update or create plot element ---
            if (t == botTopic)
                % Update my bot's position (REQUIRES IMPLEMENTATION)
                cx_bot = robotX;
                cy_bot = robotY;
                ct_bot = robotTheta;

                R = [cos(ct_bot) -sin(ct_bot); sin(ct_bot) cos(ct_bot)];
                rotated_bot = R * [limoX; limoY];

                set(hBot, 'XData', rotated_bot(1,:) + cx_bot, ...
                 'YData', rotated_bot(2,:) + cy_bot);
                
            elseif isKey(robotPlots,t)

                % ---- UPDATE EXISTING LIMO ----
            
                handles = robotPlots(t);
                hRect   = handles.rect;
                hArrowA = handles.arrowA;
                hArrowB = handles.arrowB;
            
                % orientation + body
                R = [cos(robotTheta) -sin(robotTheta); sin(robotTheta) cos(robotTheta)];
                rotated = R * [limoX; limoY];
            
                set(hRect,'XData',rotated(1,:)+robotX,'YData',rotated(2,:)+robotY);
            
                % back edge local
                bottomLeft_local  = [-L/2; -W/2];
                bottomRight_local = [-L/2;  W/2];
            
                % rotate & shift
                bottomLeft_world  = R * bottomLeft_local;
                bottomRight_world = R * bottomRight_local;
            
                backX_a = bottomLeft_world(1)  + robotX;
                backY_a = bottomLeft_world(2)  + robotY;
                backX_b = bottomRight_world(1) + robotX;
                backY_b = bottomRight_world(2) + robotY;
            
                % arrows
                
                arrow_dx = arrowLength * cos(robotTheta);
                arrow_dy = arrowLength * sin(robotTheta);
            
                set(hArrowA,'XData',backX_a,'YData',backY_a,...
                            'UData',arrow_dx,'VData',arrow_dy);
            
                set(hArrowB,'XData',backX_b,'YData',backY_b,...
                            'UData',arrow_dx,'VData',arrow_dy);

                %check for intersection
               if quiverIntersectsLine(hArrowA, lookOutX, lookOutY) || quiverIntersectsLine(hArrowB, lookOutX, lookOutY)
                   intersection = 1;
                   set(hArrowA, 'Color', 'r');
                   set(hArrowB, 'Color', 'r');
               else
                   set(hArrowA, 'Color', 'g'); % Reset color if no intersection
                   set(hArrowB, 'Color', 'g'); % Reset color if no intersection
               end
            
            else
                % ---- CREATE NEW LIMO ----
            
                % orientation for the initial drawing
                R = [cos(robotTheta) -sin(robotTheta); sin(robotTheta) cos(robotTheta)];
                rotated = R * [limoX; limoY];
            
                hRect   = fill(rotated(1,:), rotated(2,:), 'y');
                hArrowA = quiver(0,0,0,0,'LineWidth',1,'Color','g','MaxHeadSize',0.5);
                hArrowB = quiver(0,0,0,0,'LineWidth',1,'Color','g','MaxHeadSize',0.5);
            
                set(hRect,   'DisplayName', sprintf('limo_%s', name));
                set(hArrowA, 'DisplayName', sprintf('arrow_%s_a', name));
                set(hArrowB, 'DisplayName', sprintf('arrow_%s_b', name));
            
                robotPlots(t) = struct( ...
                    'rect',   hRect, ...
                    'arrowA', hArrowA, ...
                    'arrowB', hArrowB ...
                );
            end

        end
    end

    % Choose Target Point based on position and check if path recalibration
    % needed

    % Choose Target Point based on position and check if path recalibration
    % needed

    targetX = fullX(targetPointNum);
    targetY = fullY(targetPointNum);
    if dist(cx_bot, cy_bot, targetX, targetY) < tol
        targetPointNum = targetPointNum + 1;
    end
    if(targetPointNum >= length(fullX))
        if dist(cx_bot, cy_bot, targetX, targetY) < tol
            atGoal = 1;
        end
        targetPointNum = targetPointNum - 1;
    end
    
    if(targetPointNum + lookOutDist >= length(fullX))
        lookOutX = fullX(targetPointNum - 1: end);
        lookOutY = fullY(targetPointNum - 1: end);
    else
        lookOutX = fullX(targetPointNum:targetPointNum + lookOutDist);
        lookOutY = fullY(targetPointNum:targetPointNum + lookOutDist);
    end
    set(hLookOut, 'XData', lookOutX, 'YData', lookOutY);

    % recalibrate path
    if dist(cx_bot, cy_bot, targetX, targetY) > recalDist
        disp("OMG I NEED TO RECALIBRATE");
        currentPos = [cx_bot, cy_bot, ct_bot];
        [fullX, fullY] = dubinsPathToPosition(currentPos, goal, Rmin);
        set(hPath, 'XData', fullX, 'YData', fullY);
        targetPointNum = 1;
    end

    % WAIT IF THERE IS INTERSECTION
    if intersection
        v_bot = -1; omega_bot = 0.5;
    else
        [v_bot, omega_bot] = bot_step_sim(cx_bot, cy_bot, ct_bot, targetX, targetY, Rmin, Vmax, 0); 
    end
    sendCmd(tcpipClient, v_bot, omega_bot)

    [newX_bot, newY_bot, newTheta_bot] = dubins_step(cx_bot, cy_bot, ct_bot, omega_bot, v_bot, commandInterval);

    R = [cos(newTheta_bot) -sin(newTheta_bot); sin(newTheta_bot) cos(newTheta_bot)];
    rotated_bot = R * [limoX; limoY];

    drawnow limitrate;
    pause(commandInterval);
    
end

disp("PATH COMPLETE!")