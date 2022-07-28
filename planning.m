function waypoints = planning(algo,n,pathsmoothing)
% This function uses RRT, RRT*, BIRRT or PRM to produce a path and 
% returns it

% Set RNG seed for repeatable result
a = rng(5,"twister");

global map3d qstart3 qgoal3;
omap = map3d;
omap.OccupiedThreshold = 0.65;
omap.FreeThreshold = 0.65;


startPose = [qstart3(1:3) pi/3];
goalPose = [qgoal3(1:3) pi/3];
% 
% disp(getOccupancy(omap, startPose(1:3)));
% disp(getOccupancy(omap, goalPose(1:3)));

ss = ExampleHelperUAVStateSpace("MaxRollAngle",pi/3,...
                                "AirSpeed",10,...
                                "FlightPathAngleLimit",[-pi/6 pi/6],...
                                "Bounds",[0 40; 0 40; 0 10; -pi pi]);

threshold = [(goalPose-0.5)' (goalPose+0.5)'; -pi pi];
setWorkspaceGoalRegion(ss,goalPose,threshold)

sv = validatorOccupancyMap3D(ss,"Map",omap);
sv.ValidationDistance = 0.1;

pathfound = 0;

while(~pathfound)
    % RRT
    if isequal(algo,'RRT')
        planner = plannerRRT(ss,sv);
        planner.MaxConnectionDistance = 10;
        planner.MaxNumTreeNodes = 10000;
        planner.GoalBias = 0.1;  
        planner.MaxIterations = 100000;
        planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 0.1);
        % planner.EnableConnectHeuristic = true;
        [pthObj,solnInfo] = plan(planner,startPose,goalPose);
        
    
    % RRT*
    elseif isequal(algo,'RRT*')
        planner = plannerRRTStar(ss,sv);
        planner.MaxConnectionDistance = 10;
        planner.MaxNumTreeNodes = 10000;
        planner.GoalBias = 0.1;  
        planner.MaxIterations = 10000;
        planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 0.1);
        % planner.ContinueAfterGoalReached = true;
        % planner.EnableConnectHeuristic  = true;
        [pthObj,solnInfo] = plan(planner,startPose,goalPose);

    
    % BiRRT
    elseif isequal(algo,'BiRRT')

        planner = plannerBiRRT(ss,sv);
        planner.MaxConnectionDistance = 10;
        planner.MaxNumTreeNodes = 10000;
        planner.MaxIterations = 1000000;
        planner.EnableConnectHeuristic = true;
        
        [pthObj,solnInfo] = plan(planner,startPose,goalPose);
 
    % PRM
    elseif isequal(algo,'PRM')

        planner = plannerPRM(ss,sv,"MaxNumNodes",300);
        
        [pthObj,solnInfo] = plan(planner,startPose,goalPose);

    
    else
        disp("option isn't correct")
        return
    end  
    pathfound = solnInfo.IsPathFound;
end



if (pathsmoothing && ~isequal(algo,'BiRRT'))

    smoothWaypointsObj = exampleHelperUAVPathSmoothing(ss,sv,pthObj);

    
    interpolatedSmoothWaypoints = copy(smoothWaypointsObj);
    if n < smoothWaypointsObj.NumStates
        interpolate(interpolatedSmoothWaypoints,smoothWaypointsObj.NumStates)
    else
        interpolate(interpolatedSmoothWaypoints,n)
    end
    waypoints = interpolatedSmoothWaypoints.States;
else
    interpolatedPathObj = copy(pthObj);
    if n < pthObj.NumStates
        interpolate(interpolatedPathObj,pthObj.NumStates)
    else
        interpolate(interpolatedPathObj,n)
    end
    waypoints = interpolatedPathObj.States;
end

end


