function [fullX, fullY] = dubinspath(start, goal, R)

% Unpack start and goal
x0 = start(1);  y0 = start(2);  th0 = start(3);
xg = goal(1);   yg = goal(2);

% Transform goal into start frame
dx = xg - x0;
dy = yg - y0;

cs = cos(th0);  ss = sin(th0);
x =  (dx*cs + dy*ss) / R;
y = (-dx*ss + dy*cs) / R;

% Compute shortest path
[path, L] = dubins_shortest(x, y);

% Sample
N = 400;
s = linspace(0, L, N);
X = zeros(1,N);
Y = zeros(1,N);

for i = 1:N
    [xi, yi] = dubins_eval(path, s(i));
    % Rotate back to world frame
    X(i) = x0 + xi*cs - yi*ss;
    Y(i) = y0 + xi*ss + yi*cs;
end

fullX = X; 
fullY = Y;

end

function [best, L] = dubins_shortest(x, y)
    types = {'LSL','RSR','LSR','RSL','RLR','LRL'};
    best = [];
    L = inf;

    for k = 1:6
        [ok, t, u, v] = dubins_params(x, y, types{k});
        if ok
            Lcand = t + u + v;
            if Lcand < L
                L = Lcand;
                best.type = types{k};
                best.t = t; best.u = u; best.v = v;
            end
        end
    end
end

function [ok, t, u, v] = dubins_params(x, y, type)
ok = true;

switch type
    case 'LSL'
        tmp = atan2(y, x-1);
        t = mod(tmp, 2*pi);
        u = sqrt((x-1)^2 + y^2);
        v = mod(-tmp, 2*pi);

    case 'RSR'
        tmp = atan2(-y, x+1);
        t = mod(tmp, 2*pi);
        u = sqrt((x+1)^2 + y^2);
        v = mod(-tmp, 2*pi);

    case 'LSR'
        d2 = (x-1)^2 + y^2;
        if d2 < 4, ok=false; t=0; u=0; v=0; return; end
        u = sqrt(d2-4);
        t = mod(atan2(y, x-1) - atan2(2, u), 2*pi);
        v = mod(t - atan2(-y, u+2), 2*pi);

    case 'RSL'
        d2 = (x+1)^2 + y^2;
        if d2 < 4, ok=false; t=0; u=0; v=0; return; end
        u = sqrt(d2-4);
        t = mod(-atan2(y, x+1) + atan2(2, u), 2*pi);
        v = mod(-t + atan2(y, u+2), 2*pi);

    case 'RLR'
        d = sqrt(x^2 + y^2);
        if d > 4, ok=false; t=0; u=0; v=0; return; end
        phi = acos(d/4);
        t = mod(atan2(y, x) + phi, 2*pi);
        u = mod(2*pi - 2*phi, 2*pi);
        v = mod(atan2(y, x) + phi - t, 2*pi);

    case 'LRL'
        d = sqrt(x^2 + y^2);
        if d > 4, ok=false; t=0; u=0; v=0; return; end
        phi = acos(d/4);
        t = mod(-atan2(y, x) + phi, 2*pi);
        u = mod(2*pi - 2*phi, 2*pi);
        v = mod(-atan2(y, x) + phi - t, 2*pi);
end
end

function [x, y] = dubins_eval(path, s)

t = path.t; 
u = path.u; 
v = path.v;

type = path.type;

if s < t
    % First turning arc
    [x, y] = arc(s, type(1));
    return;
end

if s < t + u
    % Straight segment
    [x0, y0] = arc(t, type(1));  % end of first arc
    th0 = heading(type(1), t);
    ds = s - t;

    x = x0 + ds * cos(th0);
    y = y0 + ds * sin(th0);
    return;
end

% Final arc
s2 = s - (t + u);

% Pose at start of final arc
[x1, y1] = arc(t, type(1));      % end of first arc
th1 = heading(type(1), t);

x2 = x1 + u * cos(th1);          % end of straight
y2 = y1 + u * sin(th1);

th2 = heading(type(1), t) + u*0; % same heading at start of last arc

% Apply final arc starting at (x2, y2), heading th2
dir = type(end);

if dir == 'L'
    th = th2 + s2;
    x = x2 + ( sin(th) - sin(th2) );
    y = y2 + ( cos(th2) - cos(th) );
else
    th = th2 - s2;
    x = x2 + ( sin(th2) - sin(th) );
    y = y2 + ( cos(th) - cos(th2) );
end
end

function [x, y] = arc(s, dir)
if dir=='L'
    x = sin(s);
    y = 1-cos(s);
else
    x = sin(-s);
    y = 1-cos(s);
end
end

function th = heading(dir, s)
if dir=='L'
    th = s;
else
    th = -s;
end
end

function [x, y] = arc_from(x0, y0, dir, s, th0)
if dir=='L'
    th = th0 + s;
    x = x0 + (sin(th) - sin(th0));
    y = y0 + (cos(th0) - cos(th));
else
    th = th0 - s;
    x = x0 + (sin(th0) - sin(th));
    y = y0 + (cos(th) - cos(th0));
end
end

start = [1, 1, deg2rad(30)];
goal  = [2, 10];
Rmin  = 1.0;

[fullX, fullY] = dubinspath(start, goal, Rmin);

plot(fullX, fullY, 'LineWidth', 2)
xlim([0 10]);
ylim([0 10]);
axis equal
grid on
