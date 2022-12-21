%% A* algorithm for path finding in a 2D map - with display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [path, is_goal_reached] = a_star(grid, ...
                                          grid_start, ...
                                          grid_goal, ...
                                          cost_to_goal_factor, ...
                                          figure_handle)
% 
%

%% Init A*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
grid_i = grid.i;
grid_j = grid.j;
grid_id = grid.id;
grid_is_free = grid.is_free;
grid_size = grid.size;

parent = zeros(grid_size) * nan;
cost_from_start = ones(grid_size) * Inf;
cost = ones(grid_size) * Inf;

start_id = grid_id(grid_start(1), grid_start(2));
goal_id = grid_id(grid_goal(1), grid_goal(2));

cost_from_start(start_id) = 0;
cost(start_id) = cost_from_start(start_id) + cost_to_goal(start_id, goal_id, grid_i, grid_j);

open_set = start_id;

%% Init display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
is_open = logical(grid_id * 0);
is_open(start_id) = 1;
is_closed = logical(grid_id * 0);
current_path = start_id;

grid_color = [0.9, 0.9, 0.9];
open_set_color = [2, 232, 255] / 255;
closed_set_color = [89, 171, 95] / 255;
current_path_color = [247, 0, 255] / 255;
final_path_color = [255, 89, 0] / 255;

figure(figure_handle);
clf;
hold on;
disp_grid_1 = plot(grid_i, grid_j, 'color', grid_color, 'Linewidth', 0.75);
disp_grid_2 = plot(grid_j, grid_i, 'color', grid_color, 'Linewidth', 0.75);
disp_nodes = plot(grid_i, grid_j, '+', 'color', grid_color, 'Linewidth', 1.5);

disp_closed = plot(grid_i(start_id), grid_j(start_id), 'o', 'color', closed_set_color, 'Linewidth', 2.5);
disp_open = plot(grid_i(start_id), grid_j(start_id), 'o', 'color', open_set_color, 'Linewidth', 2.5);
disp_current_path = plot(grid_i(start_id), grid_j(start_id), 'color', current_path_color, 'Linewidth', 1.5);

disp_obstacles = plot(grid_i(~grid_is_free), grid_j(~grid_is_free), 'ko', 'Linewidth', 3);
disp_start = plot(grid_i(start_id), grid_j(start_id), 'bo', 'Linewidth', 2);
disp_goal = plot(grid_i(goal_id), grid_j(goal_id), 'ro', 'Linewidth', 2);

daspect([1 1 1]);
disp_title = title('$\mathbf{A^{*}}$ \textbf{algorithm}', 'Interpreter', 'latex');
disp_xlabel = xlabel('$i$', 'Interpreter', 'latex');
disp_ylabel = ylabel('$j$', 'Interpreter', 'latex');
xlim([grid_i(1), grid_i(end)]);
ylim([grid_j(1), grid_j(end)]);
set(gca,'TickLabelInterpreter','latex');

clc;
disp('Searching path...');

%% A*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
is_goal_reached = false;
while ~isempty(open_set)
    % Get node of the open set with lowest cost
    [~, best_in_open_set_id] = min(cost(open_set));
    current_id = open_set(best_in_open_set_id);
    current_coord = [grid_i(current_id), grid_j(current_id)];
    
    % Stop if this node is the destination
    if current_id == goal_id
        is_goal_reached = true;
        total_path = reconstruct_path(current_id, start_id, parent);
        break;
    end
    
    % Remove current node from open set
    open_set(best_in_open_set_id) = [];
    is_open(current_id) = 0; % for display only
    is_closed(current_id) = 1; % for display only
        
    % Check neighbors
    for di = -1:1
        for dj = -1:1
            neighbor_coord = [current_coord(1)+di, current_coord(2)+dj];
            
            % Ensure that this neighbor exists - i.e. is in grid
            if all(neighbor_coord > 0) && all(neighbor_coord <= grid_size)
                neighbor_id = grid_id(neighbor_coord(1), neighbor_coord(2));
                
                % Ensure that this neighbor is not an obstacle
                if grid_is_free(neighbor_id)
                    
                    % Estimate cost from start to this neighbor with the current path
                    start_to_neighbor_cost = cost_from_start(current_id) + dist_to_neighbor(di,dj);
                    
                    % If this path to the neighbor is better than the previous path to this
                    % neighbor, replace it
                    if start_to_neighbor_cost < cost_from_start(neighbor_id)
                        parent(neighbor_id) = current_id;
                        cost_from_start(neighbor_id) = start_to_neighbor_cost;
                        cost(neighbor_id) = start_to_neighbor_cost ...
                            + cost_to_goal_factor * cost_to_goal(neighbor_id, goal_id, grid_i, grid_j);
                        
                        % Add neighbor to open set if not already part of it
                        if ~any(open_set == neighbor_id)
                            open_set = [open_set, neighbor_id];
                            is_open(neighbor_id) = 1; % for display only
                        end
                    end
                end
            end
        end
    end
    disp_closed.XData = grid_i(is_closed);
    disp_closed.YData = grid_j(is_closed);
    disp_open.XData = grid_i(is_open);
    disp_open.YData = grid_j(is_open);
    current_path = reconstruct_path(current_id, start_id, parent);
    disp_current_path.XData = grid_i(current_path);
    disp_current_path.YData = grid_j(current_path);
    
    pause(0.05);
    drawnow
end
current_path = reconstruct_path(current_id, start_id, parent);
disp_current_path.XData = grid_i(current_path);
disp_current_path.YData = grid_j(current_path);

disp_final_path = plot(grid_i(total_path), grid_j(total_path), ...
                       'color', final_path_color, 'Linewidth', 3);

if is_goal_reached
    path.id = total_path;
    path.i = grid_i(total_path);
    path.j = grid_j(total_path);
    path.cost = cost_from_start(current_id);
    disp('Path found!');
else
    path.id = [];
    path.i = [];
    path.j = [];
    path.cost = Inf;
    disp('No path found');
end
end

%% Heuristic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d = cost_to_goal(node_id, goal_id, grid_i, grid_j)
%
%

    % Chessboard
%     di = abs(grid_i(goal_id) - grid_i(node_id));
%     dj = abs(grid_j(goal_id) - grid_j(node_id));
%     d = max([di, dj]);
    
    % Manhattan
%     di = abs(grid_i(goal_id) - grid_i(node_id));
%     dj = abs(grid_j(goal_id) - grid_j(node_id));
%     d = di + dj;
    
    % Quasi euclidean
%     di = abs(grid_i(goal_id) - grid_i(node_id));
%     dj = abs(grid_j(goal_id) - grid_j(node_id));
%     r = abs(di - dj);
%     d = sqrt(2) * r + max([di-r, dj-r]);
    
    % Euclidean
    di = (grid_i(goal_id) - grid_i(node_id));
    dj = (grid_j(goal_id) - grid_j(node_id));
    d = sqrt(di^2 + dj^2);
end

%% Distance to neighbors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d = dist_to_neighbor(di, dj)
%
%

    % Chessboard
%     D = [1, 1, 1;
%          1, 0, 1;
%          1, 1, 1];
    
    % Manhattan
%     D = [2, 1, 2;
%          1, 0, 1;
%          2, 1, 2];

    % Quadi euclidean
%     sr2 = sqrt(2);
%     D = [sr2, 1, sr2;
%          1,   0, 1;
%          sr2, 1, sr2];

    % Euclidean
    sr2 = sqrt(2);
    D = [sr2, 1, sr2;
         1,   0, 1;
         sr2, 1, sr2];

    d = D(di+2, dj+2);
end

%% Path reconstruction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function total_path = reconstruct_path(current_id, start_id, parent)
    total_path = current_id;
    while current_id ~= start_id
        current_id = parent(current_id);
        total_path = [current_id, total_path];
    end
end