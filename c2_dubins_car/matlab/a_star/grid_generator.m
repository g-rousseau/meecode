%% Grid generator for A* algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid_name = "grid_1";
grid_size = [20, 20]; % Count of nodes on each axis

%% Grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[grid_i, grid_j] = meshgrid(1:grid_size(1), 1:grid_size(2));
grid_i = grid_i';
grid_j = grid_j';
grid_id = zeros(grid_size);
grid_id(:) = 1:(grid_size(1) * grid_size(2));

grid.size = grid_size;
grid.i = grid_i;
grid.j = grid_j;
grid.id = grid_id;

grid_color = [0.9, 0.9, 0.9];
fig_grid = figure;
clf;
hold on;
disp_grid_1 = plot(grid_i, grid_j, 'color', grid_color, 'Linewidth', 0.75);
disp_grid_2 = plot(grid_j, grid_i, 'color', grid_color, 'Linewidth', 0.75);
disp_nodes = plot(grid_i, grid_j, '+', 'color', grid_color, 'Linewidth', 1.5);

daspect([1 1 1]);
disp_title = title('\textbf{Grid}', 'Interpreter', 'latex');
disp_xlabel = xlabel('$i$', 'Interpreter', 'latex');
disp_ylabel = ylabel('$j$', 'Interpreter', 'latex');
xlim([grid_i(1), grid_i(end)]);
ylim([grid_j(1), grid_j(end)]);
set(gca,'TickLabelInterpreter','latex');
drawnow;

%% Obstacles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid.is_free = true(grid.size);

occupancy_handle = display_occupancy(grid);
drawnow;

disp('Left click on nodes to change their occupancy');
disp('Rigth click to end');
button = 1;
 while sum(button) <=1   % read ginputs until a mouse right-button occurs
     [x, y, button] = ginput(1);
     i = round(x);
     j = round(y);
     is_in_grid = i >= grid_i(1) && i <= grid_i(end) && j >= grid_j(1) && j <= grid_j(end);
     if button ==1 && is_in_grid
         grid.is_free(i,j) = ~grid.is_free(i,j);
         delete(occupancy_handle);
         occupancy_handle = display_occupancy(grid);
         drawnow;
     end
 end
clc;
 
%% Start position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid_start = zeros(2,0);
start_handle = plot([],[]);
drawnow;

disp('Left click on nodes to set/reset start node');
disp('Rigth click to end');
button = 1;
 while sum(button) <=1   % read ginputs until a mouse right-button occurs
     [x, y, button] = ginput(1);
     i = round(x);
     j = round(y);
     is_in_grid = i >= grid_i(1) && i <= grid_i(end) && j >= grid_j(1) && j <= grid_j(end);
     if button ==1 && is_in_grid
         grid_start = [i;j];
         delete(start_handle);
         start_handle = display_start(grid_start);
         drawnow;
     end
 end
clc;
 
%% Goal position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid_goal = zeros(2,0);
goal_handle = plot([],[]);
drawnow;

disp('Left click on nodes to set/reset goal node');
disp('Rigth click to end');
button = 1;
 while sum(button) <=1   % read ginputs until a mouse right-button occurs
     [x, y, button] = ginput(1);
     i = round(x);
     j = round(y);
     is_in_grid = i >= grid_i(1) && i <= grid_i(end) && j >= grid_j(1) && j <= grid_j(end);
     if button ==1 && is_in_grid
         grid_goal = [i;j];
         delete(goal_handle);
         goal_handle = display_goal(grid_goal);
         drawnow;
     end
 end
clc;

%% Create file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save(grid_name, 'grid', 'grid_start', 'grid_goal');
clear all; close all; clc;

%% Display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function occupancy_handle = display_occupancy(grid)
    occupancy_handle = plot(grid.i(~grid.is_free(:)), grid.j(~grid.is_free(:)), ...
                            'ko', 'Linewidth', 3);
end

function start_handle = display_start(grid_start)
    start_handle = plot(grid_start(1), grid_start(2), 'b+', 'Linewidth', 3);
end

function goal_handle = display_goal(grid_goal)
    goal_handle = plot(grid_goal(1), grid_goal(2), 'r+', 'Linewidth', 3);
end