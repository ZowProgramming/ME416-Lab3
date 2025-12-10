%% MQTT SETUP
broker = "mqtt://rasticvm.lan"; 
clientID = "YoAn_Bot";

robotID = "#"; 
topic = "rb/" + robotID;
RobotPositions = containers.Map('KeyType', 'char', 'ValueType', 'any');

botID = "limo809";
botTopic = "rb/" + botID;

cmdTopic = "cmd/" + botID;
goalTopic = "goal/" + botID; 

% Initialize robot state
robotState = "WAIT"; % Can be: WAIT, GO, STOP, HALT

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

% Subscribe to robot positions
subscribe(mqttClient, topic); 
fprintf("Subscribed to topic: %s\n", topic);

% Subscribe to command topic with callback
subscribe(mqttClient, cmdTopic, "Callback", @cmdCallback);
fprintf("Subscribed to command topic: %s\n", cmdTopic);

% Subscribe to goal topic with callback
subscribe(mqttClient, goalTopic, "Callback", @goalCallback);
fprintf("Subscribed to goal topic: %s\n", goalTopic);

%% TCP SETUP
limoAddress = '192.168.1.172';   % Set this to the Limo's IP address
limoPort    = 12345;             % Do not change

runLimo = true;

if runLimo
    tcpipClient = tcpclient(limoAddress, limoPort);
    disp('Connected to Limo.');
else
    tcpipClient = [];
end

%% PARAMETER DEFINITIONS (CONFIG)

start = [0, 0, pi];  % Define start: [x, y, theta] where theta is in radians
goal = [-3, 2.5]; % Define goal: [x, y]

Rmin = 1; % Minimum turning radius
Vmax = 3; % Maximum velocity
commandInterval = 0.1; % Time per command

limos = 0; % number of other limos

lookOutDist = 2; % how many checkpoints ahead the limo looks to see if anything is in it's path
tol = 0.1; % how close to point to be considered 'at' the point
recalDist = 0.5; % how far from goal point to recalibrate path
arrowLength = 0.75;

%% INITIALIZE PLOT

% Find start position from MQTT
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

        robotX = pos.x;
        robotY = - pos.z;

        qx = data.rot(1);
        qy = data.rot(2);
        qz = data.rot(3);
        qw = data.rot(4);
        q = [qw, qx, qy, qz];

        ROT = quat2rotm(q);
        robotTheta = atan2(ROT(3,1), ROT(1,1));

        % Store latest robot position
        RobotPositions(t) = pos;

        % --- Update or create plot element ---
        if (t == botTopic)
            start = [robotX, robotY, robotTheta];
        end
    end
end

% Compute initial path
[fullX, fullY] = dubinsPathToPosition(start, goal, Rmin);

figure;

hold on;
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % plot start point
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % plot end point
axis equal;
grid on;

% Initialize rectangle to represent Limos; L = limo length, W = limo width
L = 0.3; W = 0.2;
limoX = [-L/2, L/2, L/2, -L/2];
limoY = [-W/2, -W/2, W/2,  W/2];

% Initialize our 'bot' which is the LIMO we control
hBot = patch(limoX + start(1), limoY + start(2), 'blue','DisplayName','LIMO');

%% INITIALIZE ALL OTHER LIMOS
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
hLookOut = plot(NaN, NaN, 'ro-', 'LineWidth', 2);  % lookout segment

%% REAL-TIME PLOT UPDATING FOR SIMULATION (MAIN LOOP)

atGoal = 0;
intersection = 0;
targetPointNum = 1;

lookOutX = fullX(targetPointNum:min(targetPointNum + lookOutDist, length(fullX)));
lookOutY = fullY(targetPointNum:min(targetPointNum + lookOutDist, length(fullY)));

% Initialize position variables
cx_bot = start(1);
cy_bot = start(2);
ct_bot = start(3);

disp("Waiting for GO command...");

while ~atGoal
    % Get current robot state
    currentState = evalin('base', 'robotState');
    
    % Handle HALT command - exit gracefully
    if currentState == "HALT"
        disp("HALT command received. Stopping robot and exiting...");
        if runLimo
            sendCmd(tcpipClient, 0, 0);
        end
        break;
    end
    
    % Handle STOP command - stop but don't exit
    if currentState == "STOP"
        if runLimo
            sendCmd(tcpipClient, 0, 0);
        end
        pause(commandInterval);
        continue; % Skip the rest of the loop
    end
    
    % Handle WAIT command - update goal if received but don't move
    if currentState == "WAIT"
        if runLimo
            sendCmd(tcpipClient, 0, 0);
        end
        % Check if goal has been updated
        newGoal = evalin('base', 'goal');
        if ~isequal(newGoal, goal)
            disp("Goal updated while in WAIT state");
            goal = newGoal;
            % Update goal marker on plot
            plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            % Recalculate path when we get GO command
            [fullX, fullY] = dubinsPathToPosition([cx_bot, cy_bot, ct_bot], goal, Rmin);
            set(hPath, 'XData', fullX, 'YData', fullY);
            targetPointNum = 1;
        end
        pause(commandInterval);
        continue; % Skip the rest of the loop
    end
    
    % Only execute movement if in GO state
    if currentState ~= "GO"
        pause(commandInterval);
        continue;
    end
    
    % Read all new messages
    msg = read(mqttClient);
    intersection = 0;

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

            robotX = pos.x;
            robotY = - pos.z;

            qx = data.rot(1);
            qy = data.rot(2);
            qz = data.rot(3);
            qw = data.rot(4);
            q = [qw, qx, qy, qz];
    
            ROT = quat2rotm(q);
            robotTheta = - atan2(ROT(3,1), ROT(1,1));
            
            % Store latest robot position
            RobotPositions(t) = pos;

            % --- Update or create plot element ---
            if (t == botTopic)
                cx_bot = robotX;
                cy_bot = robotY;
                ct_bot = robotTheta;

                R = [cos(ct_bot) -sin(ct_bot); sin(ct_bot) cos(ct_bot)];
                rotated_bot = R * [limoX; limoY];

                set(hBot, 'XData', rotated_bot(1,:) + cx_bot, ...
                 'YData', rotated_bot(2,:) + cy_bot);
                
            elseif isKey(robotPlots,t)
                % ---- UPDATE EXISTING LIMO ----
                name = extractAfter(t,"rb/");
            
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

                % Check for intersection
                if quiverIntersectsLine(hArrowA, lookOutX, lookOutY) || quiverIntersectsLine(hArrowB, lookOutX, lookOutY)
                    intersection = 1;
                    set(hArrowA, 'Color', 'r');
                    set(hArrowB, 'Color', 'r');
                else
                    set(hArrowA, 'Color', 'g');
                    set(hArrowB, 'Color', 'g');
                end
            
            else
                % ---- CREATE NEW LIMO ----
                name = extractAfter(t,"rb/");

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

    % Choose Target Point based on position
    targetX = fullX(targetPointNum);
    targetY = fullY(targetPointNum);
    
    if dist(cx_bot, cy_bot, targetX, targetY) < tol
        targetPointNum = targetPointNum + 1;
    end
    
    if(targetPointNum >= length(fullX))
        if dist(cx_bot, cy_bot, goal(1), goal(2)) < tol
            atGoal = 1;
        end
        targetPointNum = length(fullX);
    end
    
    % Update lookout path
    if(targetPointNum + lookOutDist >= length(fullX))
        lookOutX = fullX(targetPointNum: end);
        lookOutY = fullY(targetPointNum: end);
    else
        lookOutX = fullX(targetPointNum:targetPointNum + lookOutDist);
        lookOutY = fullY(targetPointNum:targetPointNum + lookOutDist);
    end
    set(hLookOut, 'XData', lookOutX, 'YData', lookOutY);

    % Recalibrate path if too far off
    if dist(cx_bot, cy_bot, targetX, targetY) > recalDist
        disp("Recalibrating path...");
        currentPos = [cx_bot, cy_bot, ct_bot];
        [fullX, fullY] = dubinsPathToPosition(currentPos, goal, Rmin);
        set(hPath, 'XData', fullX, 'YData', fullY);
        targetPointNum = 1;
    end

    % Calculate velocities based on collision detection
    if intersection
        v_bot = -1; 
        omega_bot = 0.5;
    else
        [v_bot, omega_bot] = bot_step_sim(cx_bot, cy_bot, ct_bot, targetX, targetY, Rmin, Vmax, 0); 
    end
    
    if runLimo
        sendCmd(tcpipClient, v_bot, omega_bot);
    end

    drawnow;
    pause(commandInterval);
end

disp("PATH COMPLETE!")

%%% ALL FUNCTIONS

%% Callback Functions

function cmdCallback(~, message)
    % Handle command messages: WAIT, GO, STOP, HALT
    cmd = char(message.Data);
    cmd = strtrim(cmd); % Remove any whitespace
    
    fprintf("Received command: %s\n", cmd);
    
    % Validate command
    validCommands = ["WAIT", "GO", "STOP", "HALT"];
    if ismember(cmd, validCommands)
        assignin('base', 'robotState', cmd);
    else
        warning("Invalid command received: %s", cmd);
    end
end

function goalCallback(~, message)
    % Handle goal updates - only in WAIT state
    currentState = evalin('base', 'robotState');
    
    if currentState == "WAIT"
        data = jsondecode(char(message.Data));
        if isfield(data, 'goal')
            newGoal = data.goal;
            fprintf("Goal updated to: [%.2f, %.2f]\n", newGoal(1), newGoal(2));
            assignin('base', 'goal', newGoal);
        end
    else
        disp("Goal message received but ignored (not in WAIT state)");
    end
end

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
    n = 37; % number of angles checked
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

function x = rand_val(a,b)
    x = a + (b-a)*rand;
end

function d = dist(x1, y1, x2, y2)
    d = sqrt( (x2 - x1)^2 + (y2 - y1)^2 );
end

%% Simulate LIMO movement

function [xn, yn, thetan] = dubins_step(x, y, theta, omega, v, dt)
    xn      = x + v*cos(theta)*dt;
    yn      = y + v*sin(theta)*dt;
    thetan  = theta + omega*dt;
end

function [xn, yn, thetan] = move_dumb_limo_sim(x,y,theta,Rmin,Vmax, dt)
    v = rand_val(0,Vmax);
    omega = rand_val(-v/Rmin, v/Rmin);

    xn      = x + v*cos(theta)*dt;
    yn      = y + v*sin(theta)*dt;
    thetan  = theta + omega*dt;
end

function [v, omega] = bot_step_sim(limoX, limoY, limoTheta, targetX, targetY, Rmin, Vmax, atGoal)
    %% ----------------- CONFIG -----------------
    omegaMax = Vmax/Rmin;
    Kp_linear = 1.0;
    Kp_angular = 2.0;
    
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

function flag = segmentsIntersect(x1,y1,x2,y2,x3,y3,x4,y4)
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

%% Helper function for sending commands to LIMO

function sendCmd(tcpipClient, v, w)
    try
        fwrite(tcpipClient, sprintf('%.4f,%.4f', v, w));
        fprintf("SEND: %.3f, %.3f\n", v, w);
    catch
        warning('TCP write failed.');
    end
end