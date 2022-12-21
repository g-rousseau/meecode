%% Quadrotor simulation intialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1: Initialize simulation by running init.m
% 2: Run simulink simulation
% 3: Run dispay_results.m

% clear all; close all; clc;

% Drone parameters
drone_mass = 0.3; % [kg]
drone_inertia = diag([0.0007 0.0007 0.0015]); % [kg.m^2]
drone_arms_length = 0.1; % [m]

% Motors parameters
propeller_lift_coeff = 9e-07; % [N/rad/s]
propeller_drag_coeff = 6e-09; % [N.m/rad/s]
rpm_to_rad_sec = pi/30;
propeller_speed_min = 3000 * rpm_to_rad_sec; % [rad/s]
propeller_speed_max = 20000 * rpm_to_rad_sec; % [rad/s]

% Physics parameters
gravity = 9.81; % [m/s^2]

% Simulation parameters
simulation_step = 0.005; % [s]
simulation_duration = 5; % [s]

init_position_ned = [0; 0; 0]; % [m]
init_velocity_ned = [0; 0; 0]; % [m/s]
init_quaternion = [1; 0; 0; 0]; % [] unit quaternion
init_angular_velocity_body = [0; 0; 0]; % [rad/s]

roll_ref_magnitude = deg2rad(10); % [rad]
pitch_ref_magnitude = deg2rad(10); % [rad]
yaw_ref_magnitude = deg2rad(360); % [rad]

roll_ref_frequency = 0.5; % [Hz]
pitch_ref_frequency = 0.3; % [Hz]
yaw_ref_frequency = 0.05; % [Hz]

% Controller parameters
k_q = 0.05; % [N.m/rad]
k_omega = 0.007;  % [N.m.s/rad]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
drone_inertia_inv = inv(drone_inertia);
propeller_speed_min_square = propeller_speed_min^2;
propeller_speed_max_square = propeller_speed_max^2;
l = drone_arms_length;
a = propeller_lift_coeff;
b = propeller_drag_coeff;
demix_matrix = [a    a    a    a
                a*l -a*l -a*l  a*l
                a*l  a*l -a*l -a*l
                b   -b    b   -b];
mix_matrix = inv(demix_matrix);
clear l a b;