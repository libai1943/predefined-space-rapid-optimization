function [x, y,theta, path_length] = SearchViaAStar(start_config, for_path_length_flag)
global params_
grid_node_2d = cell(params_.ha.num_nodes_x, params_.ha.num_nodes_y);
iter = 0;
path_length = 0;
a_flag = 0;
x = []; y = []; theta = [];
start_config_2d = start_config(1:2);
goal_config = [params_.task.xtf, params_.task.ytf];
start_ind = ConvertXY2Ind(start_config_2d(1), start_config_2d(2));
goal_ind = ConvertXY2Ind(goal_config(1), goal_config(2));
if((params_.demo.imdilate_map(start_ind(1), start_ind(2)) == 1 ) || ...
        (params_.demo.imdilate_map(goal_ind(1), goal_ind(2)) ==1))
    return;
end
expand_pattern = [-1, 1, 1.414; -1, 0, 1;  -1, -1, 1.414; 0, 1, 1; ...
    0, -1, 1;  1, 1, 1.414;1, 0, 1;  1, -1, 1.414] * (0.5 * (params_.ha.resolution_dx + params_.ha.resolution_dy));

%  init
init_node.config = start_config_2d;
init_node.ind = start_ind;
init_node.g = 0;
init_node.h = params_.ha.multiplier_H_for_2dim_A_star * sum(abs(start_config_2d(1:2) - goal_config(1:2)));
init_node.f = init_node.h + init_node.g;
init_node.is_in_openlist = 1;
init_node.is_in_closelist = 0;
init_node.parent_id = [-999, -999];
grid_node_2d{start_ind(1), start_ind(2)} = init_node;

openlist = [init_node.ind, init_node.f];
while((~isempty(openlist)) && (~a_flag) && (iter < params_.ha.num_nodes_x ^2))
    iter = iter + 1;
    ind_tmp = find(openlist(:, 3) == min(openlist(:, 3)));
    ind_tmp = ind_tmp(end);
    ind = openlist(ind_tmp, 1 : 2);
    grid_node_2d{ind(1), ind(2)}.is_in_openlist = 0;
    grid_node_2d{ind(1), ind(2)}.is_in_closelist = 1;
    cur_node = grid_node_2d{ind(1), ind(2)};
    openlist(ind_tmp, :) = [];
    for ii = 1 : 8
        clear child_node
        child_node.config = cur_node.config + expand_pattern(ii, (1 : 2));
%         scatter(child_node.config(1), child_node.config(2));
        child_node.ind = ConvertXY2Ind(child_node.config(1), child_node.config(2));
        child_node.g = cur_node.g + expand_pattern(ii, 3) ;
        child_node.h = params_.ha.multiplier_H_for_2dim_A_star * sum(abs(goal_config - child_node.config));
        child_node.f = child_node.g + child_node.h;
        child_node.parent_id = cur_node.ind;

        if((child_node.ind(1) > params_.ha.num_nodes_x) || (child_node.ind(1) < 1) || ...
                (child_node.ind(2) > params_.ha.num_nodes_y) || (child_node.ind(2) < 1))
            continue
        end
        %  the node has been expand
        if(~isempty(grid_node_2d{child_node.ind(1), child_node.ind(2)}))
            %  in closelist
            if(grid_node_2d{child_node.ind(1), child_node.ind(2)}.is_in_closelist)
                continue
                %  compare g
            elseif(child_node.g < grid_node_2d{child_node.ind(1), child_node.ind(2)}.g - 0.1)
                %  change
                child_node.is_in_closelist = 0;
                child_node.is_in_openlist = 1;
                grid_node_2d{child_node.ind(1), child_node.ind(2)} = child_node;
                openlist(find(child_node.ind(1) == openlist(:, 1) & child_node.ind(2) == openlist(:, 2)), :) = [];
                openlist = [openlist; child_node.ind, child_node.f];
                continue
            end
        else
%             colision-free? 
%             test_x = linspace(child_node.config(1), cur_node.config(1), 4);
%             test_y = linspace(child_node.config(2), cur_node.config(2), 4);
%             test_ind = ConvertXY2Ind(test_x(2:end), test_y(2:end));
%             if(sum(params_.demo.imdilate_map(sub2ind(size(params_.demo.imdilate_map), test_ind(:, 1), test_ind(:, 2))))) 
            if(params_.demo.imdilate_map(child_node.ind(1), child_node.ind(2)) == 1)
                child_node.is_in_closelist = 1;
                child_node.is_in_openlist = 0;
                grid_node_2d{child_node.ind(1), child_node.ind(2)} = child_node;
                continue
            end
            % new
%             plot([child_node.config(1), cur_node.config(1)], [child_node.config(2), cur_node.config(2)])
            child_node.is_in_openlist = 1;
            child_node.is_in_closelist = 0;
            grid_node_2d{child_node.ind(1), child_node.ind(2)} = child_node;
            openlist = [openlist; child_node.ind, child_node.f];
            
            if(child_node.ind == goal_ind)
                a_flag = 1;
                break;
            end
        end
    end
end
% callback
cur_node = grid_node_2d{goal_ind(1), goal_ind(2)};
if(for_path_length_flag)
    path_length = cur_node.g;
    return;
end
while(cur_node.parent_id ~= [-999, -999])
    x = [cur_node.config(1), x];
    y = [cur_node.config(2), y];
%     scatter(cur_node.config(1), cur_node.config(2))
    cur_node = grid_node_2d{cur_node.parent_id(1), cur_node.parent_id(2)};
end
x = [start_config(1), x];
y = [start_config(2), y];
% plot(x,y);

theta = Reformulate(x, y);
end

function theta = Reformulate(x, y)
theta = zeros(1, length(x));
for ii = 1 : length(x)
    if(x(ii) ~= 0)
        theta(ii) = atan(y(ii) / x(ii));
    elseif(y(ii) >= 0)
        theta(ii) = pi/2;
    else
        theta(ii) = 3 * pi / 2;
    end
end
theta = RegulateAngle(theta);
end

function angle = RegulateAngle(angle)
for ii = 1 : length(angle)
    while(angle(ii) >= 2 *pi)
        angle(ii) = angle(ii) - 2 * pi;
    end
    while(angle(ii) < 0)
        angle(ii) = angle(ii) + 2 * pi;
    end
end
end

function ind = ConvertXY2Ind(x, y)
global params_
indx = floor((x  - params_.demo.xmin) / params_.ha.resolution_dx + 0.001) + 1;
indy = floor((y  - params_.demo.ymin) / params_.ha.resolution_dy + 0.001) + 1;
ind = [indx, indy];
end
