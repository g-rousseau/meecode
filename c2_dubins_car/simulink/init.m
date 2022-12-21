%% Dubins car simulation intialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 1.3: Dubins car modeling
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1: Initialize simulation by running init.m
% 2: Run either simulation.slx or simulation_no_simulink.m
% 3: Run display_results.m

% clear all; close all; clc;

init_position = [0; 0]; % [m]
init_angle = 0; % [m]

speed_max = 5; % [m/s]
angular_speed_max = deg2rad(45); % [rad/s]

simulation_duration = 60; % [s]
simulation_step = 0.1; % [s]

x_ref_mag = 7; % [m]
y_ref_mag = 3; % [m]
x_ref_frequency = 0.025; % [Hz]
y_ref_frequency = 0.05; % [Hz]
y_ref_phase = 0; % [rad]

map_bounds = 10; % [m]
dead_zone = 0.6; % []