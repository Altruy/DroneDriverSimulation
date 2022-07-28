function slid = slide_window(position,mode)
% Slide window updates the windows according to the nearest waypoints to
% the uav
% mode 0 uses Euclidean didtance however mode 1 uses 
% difference in heading angles
slid = 0;
global waypoints w1 w2 del_g uav_pose flag wp_N obs wp_dist;
if mode == 0
    for c = 1:3
        dis = sqrt((position(1)-waypoints(w1:w2,1)).^2 ...
            + (position(2)-waypoints(w1:w2,2)).^2 ...
            + (position(3)-waypoints(w1:w2,3)).^2) ;
        if length(dis) < 2
            return
        end
        if (dis(2) < dis(1) || dis(1) < del_g || obs == 1 ) && (dis(2) < 3* wp_dist)  % time to slide
            if w2 + 1 <= wp_N 
                w2 = w2 + 1;
                w1 = w1 + 1;
                slid = 2; % disp('slid both')
            elseif w1 + 1 <= w2
                w1 = w1 + 1; 
                slid = 1 ;%   disp('slid one one')
            else 
                slid = -1 ;% cant slide either, only goal point in wondow
            end
        else
            break
        end
    end
else
    if flag > 3
        slid = -1;
        return
    end
     if ~isnan(uav_pose) 
        dis   = angle_forslide(position);
        if length(dis) < 2
         return
        end
        if dis(2) < dis(1) || flag > 5 || obs == 1 % time to slide
            if w2 + 1 <=  wp_N 
                w2 = w2 + 1;
                w1 = w1 + 1;
                slid = 2; %disp('slid both')
            elseif w1 + 1 <= w2
                w1 = w1 + 1; 
                slid = 1 ;%   disp('slid one one')
            else 
                slid = -1 ;% cant slide either, only goal point in wondow
            end
        end
     end
end
end

