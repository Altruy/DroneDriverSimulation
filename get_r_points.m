function p = get_r_points(m)
%GET_R_POINTS Summary of this function goes here
%   Detailed explanation goes here
if isequal(m,'start')
    x = floor(randi([0 4]));
    y = floor(randi([5 25]));
    z = 2;
elseif isequal(m,'goal')
    x = floor(randi([27 35]));
    y = floor(randi([5 18]));
    z = 3;
end
p = [x y z];

