
function main(algo)
load map3D
% Global Parameters Declarations -----------------
global sensor_range;  % Determines limited sensor range
global uav_pose flag;  % keeps trace of sensor pose
global arena_limits;  % Boundaries of the arena: [xmin xmax ymin ymax]
global arena_map;     % Description of obstacles in the environment
global infinity;      % Large value to be used as 'infinity'
global qstart3 qgoal3 currg;  % Start and goal configurations
global max_robspeed;
global min_robspeed;
global del_g;    % delta, reached distance
global safety;
global map3d waypoints wp_N ;
global m_line_quat v_quat;
global w1 w2 winsize;
global obs ; %flag for obstacle
global planAlgo ; % algorithm for path planning
global wp_dist;
sensor_range = 5;
infinity = 1e5;
arena_limits = [0 40 0 40];
arena_map = [];
max_robspeed = 1;
min_robspeed = 0.8;
del_g = 0.5; 
safety = 0.5;
map3d = map3D;
winsize = 3;
planAlgo = algo;
% qstart3 = [5 5 1];
% qgoal3 = [27 10 3];
qstart3 = get_r_points('start')
qgoal3 = get_r_points('goal')

% planning algo: RRT, RRT*, BiRRT, PRM
% number of way points
% path smoothing, 1 || 0
waypoints = planning(planAlgo,20,1);
wp_dist = get_dist(waypoints(1,:),waypoints(2,:));
wp_N = length(waypoints);
add_obstacle(map3d)
draw_fig() 
step_iter(200);

% 
% waypoints = planning('RRT*',15,1);
% wp_N = length(waypoints);
% add_obstacle(map3d)
% draw_fig(2)
% step_iter(200);
% 
% waypoints = planning('BiRRT',15,1);
% wp_N = length(waypoints);
% add_obstacle(map3d)
% draw_fig(3)
% step_iter(200);
% 
% waypoints = planning('PRM',18,1);
% wp_N = length(waypoints);
% add_obstacle(map3d)
% draw_fig(4)
% step_iter(200);
% 




