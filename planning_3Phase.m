% Set RNG seed for repeatable result
% rng(5,"twister");

% omap = occupancyMap3D(.9346);
% file = fopen('env.txt', 'r');
% formatSpec = '%d %d %d';
% sizeA = [3 Inf];
% A = fscanf(file,formatSpec,sizeA);
% A = A.';
% temp = A(:,2);
% A(:,2) = A(:,3);
% A(:,3) = temp;
% 
% pose = [ 0 0 0 1 0 0 0];
% maxRange = 538;
% 
% insertPointCloud(omap, pose, A, maxRange);
% % omap.FreeThreshold = omap.OccupiedThreshold;
% 
% 
% for i = A.'
%     updateOccupancy(omap, i.', 1.0);
% end
% for i = 1:262
%     for j = 1:538
%         updateOccupancy(omap, [i, j, 6], 1.0);
%         updateOccupancy(omap, [i, j, 7], 1.0);
%     end
%     
% end
% vehicleRadius = 1;
% safetyRadius = 1;
% inflationRadius = vehicleRadius + safetyRadius;
% inflate(omap, inflationRadius);
% 
% omap.OccupiedThreshold = 0.999;
% omap.FreeThreshold = 0.998;
% map3d = load('map3d.mat');
% omap = map3d.map3D;
% 
% omap.show();


mapData = load("omap.mat","omap");
omap = mapData.omap;
% Consider unknown spaces to be unoccupied
omap.OccupiedThreshold = 0.999;
omap.FreeThreshold = 0.998;

% omap = load('map3d.mat');
% omap = omap.map3D;
% omap.show();

% 
% omap.OccupiedThreshold = 0.65;
% omap.FreeThreshold = 0.65;

%%

startPose = [122.8 337 35 1.208];
goalPose = [180 457 11.5 1.208];

disp(getOccupancy(omap, startPose(1:3)));
disp(getOccupancy(omap, goalPose(1:3)));

ss = ExampleHelperUAVStateSpace("MaxRollAngle",pi/3,...
                                "AirSpeed",8,...
                                "FlightPathAngleLimit",[-pi/6 pi/6],...
                                "Bounds",[0 262; 0 538; 0 70; -pi pi]);


threshold = [(goalPose-0.5)' (goalPose+0.5)'; -pi pi];
setWorkspaceGoalRegion(ss,goalPose,threshold)

sv = validatorOccupancyMap3D(ss,"Map",omap);
sv.ValidationDistance = 0.1;

prompt = 'Which path planning algorithm you want to use? \n1. RRT\n2. RRT*\n3. BiRRT\n4. PRM\n';
SelectedAlgorithm = input(prompt,"s");

%% RRT
if isequal(SelectedAlgorithm,'1')|| isequal(SelectedAlgorithm,'RRT')
    tic
    disp('Entered into RRT')
    [user,sys] = memory;
    planner = plannerRRT(ss,sv);
    planner.MaxConnectionDistance = 10;
    planner.MaxNumTreeNodes = 10000;
    planner.GoalBias = 0.1;  
    planner.MaxIterations = 100000;
    planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 0.1);
    % planner.EnableConnectHeuristic = true;
    
    [pthObj,solnInfo] = plan(planner,startPose,goalPose);
    [user2,sys2] = memory;
    disp('memory required in path planning');
    memory_used_in_bytes=user2.MemAvailableAllArrays-user.MemAvailableAllArrays;
    disp(memory_used_in_bytes)
    disp('time required in Path planning');
    toc

%% RRT*
elseif isequal(SelectedAlgorithm,'2') || isequal(SelectedAlgorithm,'RRT*')
    tic
    disp('Entered into RRT*')
    [user,sys] = memory;
    planner = plannerRRTStar(ss,sv);
    planner.MaxConnectionDistance = 10;
    planner.MaxNumTreeNodes = 10000;
    planner.GoalBias = 0.1;  
    planner.MaxIterations = 10000;
    planner.GoalReachedFcn = @(~,x,y)(norm(x(1:3)-y(1:3)) < 0.1);
    % planner.ContinueAfterGoalReached = true;
    % planner.EnableConnectHeuristic  = true;
    
    [pthObj,solnInfo] = plan(planner,startPose,goalPose);
    [user2,sys2] = memory;
    disp('memory required in path planning');
    memory_used_in_bytes=user2.MemAvailableAllArrays-user.MemAvailableAllArrays;
    disp(memory_used_in_bytes)
    disp('time required in Path planning');
    toc

%% BiRRT
elseif isequal(SelectedAlgorithm,'3') || isequal(SelectedAlgorithm,'BiRRT')
    tic
    disp('Entered into BiRRT')
    [user,sys] = memory;
    planner = plannerBiRRT(ss,sv);
    planner.MaxConnectionDistance = 10;
    planner.MaxNumTreeNodes = 10000;
    planner.MaxIterations = 10000;
    planner.EnableConnectHeuristic = true;
    
    [pthObj,solnInfo] = plan(planner,startPose,goalPose);
    [user2,sys2] = memory;
    disp('memory required in path planning');
    memory_used_in_bytes=user2.MemAvailableAllArrays-user.MemAvailableAllArrays;
    disp(memory_used_in_bytes)
    disp('time required in Path planning');
    toc


%% PRM
elseif isequal(SelectedAlgorithm,'4') || isequal(SelectedAlgorithm,'PRM')
    tic
    disp('Entered into PRM')
    [user,sys] = memory;
    planner = plannerPRM(ss,sv,"MaxNumNodes",300);
    
    [pthObj,solnInfo] = plan(planner,startPose,goalPose);
    [user2,sys2] = memory;
    disp('memory required in path planning');
    memory_used_in_bytes=user2.MemAvailableAllArrays-user.MemAvailableAllArrays;
    disp(memory_used_in_bytes)
    disp('time required in Path planning');
    toc

else
    disp("option isn't correct")
    return
end  

new_path = navPath(ss);
new_path.append([34 10 9 1.57;... 
    34 40 9 1.57; ...
    34 140 35 1.208]);
new_path_smoothed = copy(new_path);
new_path.append(pthObj.States)

if (solnInfo.IsPathFound)
    figure("Name","OriginalPath")
    % Visualize the 3-D map
    show(omap)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
    scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
    
%     interpolatedPathObj = copy(pthObj);
    interpolatedPathObj = copy(new_path);
    interpolate(interpolatedPathObj,1000)
    
    % Plot the interpolated path based on UAV Dubins connections
    hReference = plot3(interpolatedPathObj.States(:,1), ...
        interpolatedPathObj.States(:,2), ...
        interpolatedPathObj.States(:,3), ...
        "LineWidth",2,"Color","g");
    
    % Plot simulated UAV trajectory based on fixed-wing guidance model
    % Compute total time of flight and add a buffer
    timeToReachGoal = 1.05*pathLength(new_path)/ss.AirSpeed;
    waypoints = interpolatedPathObj.States;
%     waypoints = interpolate([startPose(1),startPose(2),startPose(3),startPose(4);...
%         goalPose(1),goalPose(2),goalPose(3),goalPose(4)]);
    [xENU,yENU,zENU] = exampleHelperSimulateUAV(waypoints,ss.AirSpeed,timeToReachGoal);
    hSimulated = plot3(xENU,yENU,zENU,"LineWidth",2,"Color","r");
    legend([hReference,hSimulated],"Reference","Simulated","Location","best")
    hold off
    view([-31 63])
end

if (solnInfo.IsPathFound)
    
    
    tic
    [user,sys] = memory;
    smoothWaypointsObj = exampleHelperUAVPathSmoothing(ss,sv,pthObj);
    [user2,sys2] = memory;
    disp('memory required in path smoothing');
    memory_used_in_bytes=user2.MemAvailableAllArrays-user.MemAvailableAllArrays;
    disp(memory_used_in_bytes)
    disp('time required in Path smoothing');
    toc
    new_path_smoothed.append(smoothWaypointsObj.States)

    figure("Name","SmoothedPath")
    % Plot the 3-D map
    show(omap)
    hold on
    scatter3(startPose(1),startPose(2),startPose(3),30,"red","filled")
    scatter3(goalPose(1),goalPose(2),goalPose(3),30,"green","filled")
    
    interpolatedSmoothWaypoints = copy(new_path_smoothed);
    interpolate(interpolatedSmoothWaypoints,1000)
    
    % Plot smoothed path based on UAV Dubins connections
    hReference = plot3(interpolatedSmoothWaypoints.States(:,1), ...
        interpolatedSmoothWaypoints.States(:,2), ...
        interpolatedSmoothWaypoints.States(:,3), ...
        "LineWidth",2,"Color","g");
    
    % Plot simulated flight path based on fixed-wing guidance model
    waypoints = interpolatedSmoothWaypoints.States;
    timeToReachGoal = 1.05*pathLength(new_path_smoothed)/ss.AirSpeed;
    [xENU,yENU,zENU,a,b] = exampleHelperSimulateUAV(waypoints,ss.AirSpeed,timeToReachGoal);
    hSimulated = plot3(xENU,yENU,zENU,"LineWidth",2,"Color","r");
    
    legend([hReference,hSimulated],"SmoothedReference","Simulated","Location","best")
    hold off
    view([-31 63]);
end
