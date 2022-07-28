function [ang,ang_yaw,ang_pitch] = angle_forslide(position)
%ANGLE_FORSLIDE Summary of this function goes here
%   Detailed explanation goes here
global waypoints w1 w2 uav_pose;
ang = [];
ang_yaw = [];
ang_pitch = [];
for i = w1:w2
    [y,~] = get_mLine(position,waypoints(i,:));
    [yu,pu,~] = quat2angle(uav_pose);
    [yw,pw,~] = quat2angle(y);
    a = yu - yw ;
    b = pu - pw ;
    ang = [ang sqrt(a*a)+sqrt(b*b)];
    ang_yaw = [ang_yaw sqrt(a*a)];
    ang_pitch = [ang_pitch sqrt(b*b)];
end
end

