function path_taken = step_iter(maxSteps)
%STEPS Summary of this function goes here
%   Detailed explanation goes here
global qstart3 qgoal3 uav_pose currg waypoints v_quat m_line_quat w1 w2 winsize infinity obs flag

w1 = 1;
w2 = winsize;
flag = 0;
obs = 0;
path_taken = [];
currs = qstart3 ;
for in=1:maxSteps
    slid = slide_window(currs,0);
%     disp([slid '|' w1 w2 '|' currs])
    currg = waypoints(w1,:);
    if currs(1:3) == currg(1:3)
        disp('Reached Destination')
        hold on
        plotTransforms(qgoal3(1:3),uav_pose(1:end),'FrameSize',1,'MeshFilePath','fixedwing.stl'); 
        hold off
        drawnow;
        break
    end
    [temp,tempv] = get_mLine(currs,currg);
    if ~isnan(temp(1))
        m_line_quat = temp;
        v_quat = tempv;
    end
    [d,heading]  = real_sensor(currs);
    if d == infinity
        disp('Reached Destination')
        hold on
        plotTransforms(qgoal3(1:3),uav_pose(1:end),'FrameSize',1,'MeshFilePath','fixedwing.stl'); 
        hold off
        drawnow;
        break
    end
    if size(intersect(m_line_quat,uav_pose,'rows'),1) == 0
        flag = flag +1;
%         slid = slide_window(currs,1);
    else
        flag = 0;
    end
%     slid = slide_window(currs,0);
    
    currs = heading ;
    path_taken = [path_taken heading];
    pause(0.5)
end  

end

