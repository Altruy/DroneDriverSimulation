function draw_fig()
%DRAW_FIG Summary of this function goes here
%   Detailed explanation goes here
global map3d qstart3 qgoal3 waypoints planAlgo
% figure()
% clf;
hold on 
show(map3d)
scatter3(qstart3(1),qstart3(2),qstart3(3),20,"red","filled")  % Plot Init. Pos.
scatter3(qgoal3(1),qgoal3(2),qgoal3(3),20,"green","filled")   % Plot Goal

drawnow;

if isequal(planAlgo,'RRT')

%     show(map3d)
%     scatter3(qstart3(1),qstart3(2),qstart3(3),20,"red","filled")  % Plot Init. Pos.
%     scatter3(qgoal3(1),qgoal3(2),qgoal3(3),20,"green","filled")   % Plot Goal

    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), "*-g");  % PLot waypoints
    legend({'World','Start','Goal','RRT'},'Location','southwest')

elseif isequal(planAlgo,'RRT*')
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), "*-r");  % PLot waypoints
    legend({'World','Start','Goal','RRT*'},'Location','southwest')

elseif isequal(planAlgo,'BiRRT')
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), "*-blue");  % PLot waypoints
    legend({'World','Start','Goal','BiRRT'},'Location','southwest')

elseif isequal(planAlgo,'PRM')
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), "*-black");  % PLot waypoints
    legend({'World','Start','Goal','PRM'},'Location','southwest')

end
hold off
xlabel('X Coordinate')
ylabel('Y Coordinate')
title('Drone Driver Simulation')
grid on
drawnow;
end

