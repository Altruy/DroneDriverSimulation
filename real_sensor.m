function [dist, heading]= real_sensor(position)
global obs sensor_range infinity map3d m_line_quat v_quat qgoal3 uav_pose;
quat = m_line_quat;
dist = infinity;
heading = infinity;
numRays = 31;
cen = 16;
n = cen ;
angles = linspace(-pi/2,pi/2,numRays);
directions = [cos(angles); sin(angles); zeros(1,numRays)]';
sensorPose = [position(1) position(2) position(3) quat(1) quat(2) quat(3) quat(4)];
vertical_pose = [position(1) position(2) position(3) v_quat(1) v_quat(2) v_quat(3) v_quat(4)];
[intersectionPts,isOccupied] = rayIntersection(map3d,sensorPose,directions,sensor_range);
[intersectionPtsv,isOccupiedv] = rayIntersection(map3d,vertical_pose,directions,sensor_range);
if position(1:3) == qgoal3
    plotTransforms(sensorPose(1:3),sensorPose(4:end),...
           'FrameSize',1,'MeshFilePath','fixedwing.stl'); % Vehicle sensor pose
    drawnow;
    return 
end
if isOccupied(cen) == 1 
    obs = 1;
    dist = get_dist(position,intersectionPts(cen,:));
    for i = 1:cen-2
        if isOccupied(cen-i) < 1 && isOccupied(cen-i-1) < 1 && isOccupied(cen-i+1) < 1
            n = cen-i;
            heading = get_heading(position,intersectionPts(n,:),1);
            break
        end 
        if isOccupied(cen+i) < 1 && isOccupied(cen+i-1) < 1 && isOccupied(cen+i+1) < 1
            n = cen+i;
            heading = get_heading(position,intersectionPts(n,:),1);
            break
        end
        if isOccupiedv(cen-i) < 1 && isOccupiedv(cen-i-1) < 1 && isOccupiedv(cen-i+1) < 1
            n = cen-i;
            heading = get_heading(position,intersectionPtsv(n,:),1);
            break
        end 
        if isOccupiedv(cen+i) < 1 && isOccupiedv(cen+i-1) < 1 && isOccupiedv(cen+i+1) < 1
            n = cen+i;
            heading = get_heading(position,intersectionPtsv(n,:),1);
            break
        end
    end
else
    obs = 0;
    heading = get_heading(position,intersectionPts(cen,:),0);
    if heading(1:3) ~= qgoal3
        dist = 0;
    end

end
hold on
% new heading
% if isOccupied(cen) == 1 
%     plot3([sensorPose(1),intersectionPts(n,1)],...
%             [sensorPose(2),intersectionPts(n,2)],...
%             [sensorPose(3),intersectionPts(n,3)],'-r')
% end
% for i = 1:numRays
%     i = cen;
%     plot3([sensorPose(1),intersectionPts(i,1)],...
%       [sensorPose(2),intersectionPts(i,2)],...
%       [sensorPose(3),intersectionPts(i,3)],'-b') % Plot rays
%     if isOccupied(i) == 1
%       plot3(intersectionPts(i,1),intersectionPts(i,2),intersectionPts(i,3),'*r') % Intersection points
%     end
% for i = 1:numRays
%     plot3(intersectionPts(i,1),...
%           intersectionPts(i,2),...
%           intersectionPts(i,3),'xb') % Plot rays
%     if isOccupied(i) == 1
%         plot3(intersectionPts(i,1),intersectionPts(i,2),intersectionPts(i,3),'*r') % Intersection points
%     end
% end
% end
hold on;
uav_pose = get_mLine(position,heading);
plotTransforms(sensorPose(1:3),uav_pose(1:end),...
               'FrameSize',1,'MeshFilePath','fixedwing.stl'); % Vehicle sensor pose
hold off;
drawnow;
end