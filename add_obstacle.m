function add_obstacle(omap)
%ADD_OBSTACLE Summary of this function goes here
%   Detailed explanation goes here
% adding obstacle
global waypoints wp_N
rng("shuffle")
a = 5;
b =  wp_N - 4;
r = floor(randi([a b]));
wp = waypoints(r,:);
for x = (wp(1)*10-10):(wp(1)*10+10)
    for y = (wp(2)*10-10):(wp(2)*10+10)
        for z = (wp(3)*10-10):(wp(3)*10+10)
            updateOccupancy(omap, [x/10, y/10, z/10], 1.0);
            updateOccupancy(omap, [x/10, y/10, z/10], 1.0);
        end
    end
    
end
end

