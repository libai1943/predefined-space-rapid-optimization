function InitializeParams()
global params_
LoadCase();
%%  user
params_.user.ha_iter_max = 500;
params_.user.asd_opti = 0;
params_.user.asd_opti_gif = 0;

%%  vehicle
params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;
params_.vehicle.hypotenuse_length = hypot(params_.vehicle.length, params_.vehicle.lb);
params_.vehicle.radius = hypot(0.25 * params_.vehicle.length, 0.5 * params_.vehicle.lb); % Dual disk radius
params_.vehicle.r2p = 0.25 * params_.vehicle.length - params_.vehicle.lr;
params_.vehicle.f2p = 0.75 * params_.vehicle.length - params_.vehicle.lr;
params_.vehicle.vmax = 2.5;
params_.vehicle.amax = 1.0;
params_.vehicle.phymax = 0.75;
params_.vehicle.wmax = 0.5;
params_.vehicle.kappa_max = tan(params_.vehicle.phymax) / params_.vehicle.lw;
params_.vehicle.turning_radius_min = abs(1.0 / params_.vehicle.kappa_max);
params_.vehicle.threshold_s = (params_.vehicle.vmax^2) / params_.vehicle.amax;

%%  demo
params_.demo.xmin = -20;
params_.demo.xmax = 20;
params_.demo.ymin = -20;
params_.demo.ymax = 20;
if(params_.user.case_id == 28 || params_.user.case_id == 22)
    params_.demo.xmin = -25;
    params_.demo.xmax = 25;
    params_.demo.ymin = -25;
    params_.demo.ymax = 25;
end

%%  ha
params_.ha.resolution_dx = 0.4;
params_.ha.resolution_dy = 0.4;
params_.ha.resolution_dtheta = 0.2;
params_.ha.num_nodes_x = ceil((params_.demo.xmax - params_.demo.xmin) / params_.ha.resolution_dx) + 1;
params_.ha.num_nodes_y = ceil((params_.demo.ymax - params_.demo.ymin) / params_.ha.resolution_dy) + 1;
params_.ha.num_nodes_theta = ceil(2 * pi / params_.ha.resolution_dtheta) + 1;
params_.ha.penalty_for_backward = 1.0;
params_.ha.penalty_for_direction_change = 5.0;
params_.ha.penalty_for_steering_change = 0.00;
params_.ha.multiplier_H = 3.0;
params_.ha.multiplier_H_for_2dim_A_star = 3.0;
params_.ha.simulation_step = 1.2;

%%  opti
params_.opti.nfe = 100;
params_.dt_for_resampling = 0.001;




%% asd
params_.asd_resamp_dt = 0.1;

CreateCostmaps();
end


function LoadCase()
global params_
load([pwd, '\ParkingBenchmarks\CaseNo_', num2str(params_.user.case_id), '.mat']);
params_.task.x0 = x0;
params_.task.y0 = y0;
params_.task.theta0 = theta0;
params_.task.xtf = xtf;
params_.task.ytf = ytf;
params_.task.thetatf = thetatf;
params_.obstacle.Nobs = length(obstacles);
params_.obstacle.obs = obstacles;
end

function CreateCostmaps()
global params_
xmin = params_.demo.xmin;
ymin = params_.demo.ymin;
xmax = params_.demo.xmax;
ymax = params_.demo.ymax;
resolution_x = params_.ha.resolution_dx;
resolution_y = params_.ha.resolution_dy;

params_.demo.origional_map = zeros(params_.ha.num_nodes_x, params_.ha.num_nodes_y);
params_.demo.imdilate_map = params_.demo.origional_map;
costmap = zeros(params_.ha.num_nodes_x, params_.ha.num_nodes_y);

for ii = 1 : params_.obstacle.Nobs
    obs_vx = params_.obstacle.obs{ii}.x;
    obs_vy = params_.obstacle.obs{ii}.y;
    x_max = max(obs_vx);
    x_min = min(obs_vx);
    y_max = max(obs_vy);
    y_min = min(obs_vy);
    %   out of the demo
    if(x_max > xmax)
        x_max = xmax;
    elseif(x_min < xmin)
        x_min = xmin;
    end
    if(y_max > ymax)
        y_max = ymax;
    elseif(y_min < ymin)
        y_min = ymin;
    end
    [Nmax_x, Nmax_y] = ConvertXY2Ind(x_max, y_max);
    [Nmin_x, Nmin_y] = ConvertXY2Ind(x_min, y_min);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if(costmap(jj, kk) == 1)
                continue
            end
            x0 = xmin + (jj - 1) * resolution_x;
            y0 = ymin + (kk - 1) * resolution_y;
            x1 = xmin + jj * resolution_x;
            y1 = xmin + kk * resolution_y;
            %   left up right low
            test_vx = [linspace(x0, x0, 4), linspace(x0, x1, 4), linspace(x1, x1, 4), linspace(x0, x1, 4)];
            test_vy = [linspace(y0, y1, 4), linspace(y1, y1, 4), linspace(y0, y1, 4), linspace(y0, y0, 4)];
            if(any(inpolygon(test_vx, test_vy, obs_vx, obs_vy)))
                costmap(jj, kk) = 1;
            end
        end
    end
end
params_.demo.origional_map = costmap;
unit_resolution = 0.5 * (resolution_x + resolution_y);
r = ceil(params_.vehicle.radius / unit_resolution) + 1;
params_.demo.imid_r = r;
se = strel('disk', r);
params_.demo.imdilate_map = imdilate(costmap, se);
end

function [indx, indy] = ConvertXY2Ind(x, y)
global params_
indx = floor((x - params_.demo.xmin) / params_.ha.resolution_dx) + 1;
indy = floor((y - params_.demo.ymin) / params_.ha.resolution_dy) + 1;
end