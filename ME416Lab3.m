%% MQTT Setup
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

% Collect messages
for i = 1:10 msg = peek(mqttClient); 
    if size(msg,1)  % If any messages present 
        currentTopic = msg.Topic(i); 
        currentData = jsondecode(msg.Data{i}).pos; 
        disp(currentTopic); disp(currentData); 
        RobotPositions(currentTopic) = currentData; 
    end 
    
    pause(0.05); 
end
   
%% Real-Time Plot Setup
figure(1); clf;
hold on; grid on; axis equal;

xlabel("X (m)");
zlabel("Z (m)");
title("Real-Time LIMO Robot Positions");

xlabel("X (m)");
zlabel("Z (m)");
title("Real-Time LIMO Robot Positions (Z-Plane)");

% Adjust to your environment size
xlim([-5 5]);
zlim([-2 2]);

robotPlots = containers.Map;


%% Live Update Loop
disp("Starting live plotting... Press Ctrl+C to stop.");

while true

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

            % Store latest robot position
            RobotPositions(t) = pos;

            % --- Update or create plot element ---
            if isKey(robotPlots, t)
                pHandle = robotPlots(t);      % extract handle
                pHandle.XData = pos.x;
                pHandle.YData = pos.z;
            else
                h = scatter(pos.x, pos.z, 80, 'filled', ...
                            'DisplayName', t);
                robotPlots(t) = h;            % store handle in map
                legend show;
            end

        end
    end

    drawnow limitrate;
    pause(0.02);

end

