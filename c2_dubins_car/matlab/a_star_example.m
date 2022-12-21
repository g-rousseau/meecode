%% Example: A*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear all; close all; clc;

addpath('a_star');

% Problem definition
% use grid_generator.m to create a new test case
load('grid.mat');

% heuristic factor
cost_to_goal_factor = 1;

% A* path search
fig_a_star = figure(1);
[path, is_goal_reached] = a_star(grid, ...
                                 grid_start, ...
                                 grid_goal, ...
                                 cost_to_goal_factor, ...
                                 fig_a_star);

% path simplification
simplified_path = simplify_path(path);

