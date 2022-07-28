function point = get_heading(p0,p1,m)
%   takes the step size, gives new heading
%   Detailed explanation goes here

global max_robspeed min_robspeed currg sensor_range;
dg = get_dist(p0,currg);

if m == 0 % no obstacle infront
    if dg < sensor_range
        point = currg;
        return
    end
    step = max_robspeed;
else      % avoiding obstacle mode
    step = min_robspeed; 
end
d = get_dist(p0,p1);
dx = p1(1) - p0(1);
dy = p1(2) - p0(2);
dz = p1(3) - p0(3);

px = p0(1) + dx*step;
py = p0(2) + dy*step;
pz = p0(3) + dz*step;

point = [px py pz];

end

