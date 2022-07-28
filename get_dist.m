function dist = get_dist(position,waypoints)
% This function returns the distance between two points
dist = sqrt((position(1)-waypoints(1)).^2 + (position(2)-waypoints(2)).^2 + (position(3)-waypoints(3)).^2);
end

