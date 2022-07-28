function [quat,quatver] = get_mLine(p0,p1)
% This Function returns the updated sensor pose for UAV
% p0 pointing towards p1

dy = p1(2) - p0(2);
dx = p1(1) - p0(1);
dz = p1(3) - p0(3);

% axes rotation angles
thetaZ = atan2(dy,dx);
thetaY = -atan2(dz,sqrt(dx^2 + dy^2));

% Homogenous tranformations for axes rotations
avz = axang2tform([0 0 1 thetaZ]);
avy = axang2tform([0 1 0 thetaY]);
avx = axang2tform([1 0 0 pi/2]);

%Homogenous transformation fr translation
tfo = trvec2tform([dx dy dz]);

%Combined transformation
combined_hor =  avz*avy*tfo;
combined_ver = avz*avy*avx*tfo; % with 90 degree roll for vertical avoidance
quat=tform2quat(combined_hor);
quatver = tform2quat(combined_ver);
end

