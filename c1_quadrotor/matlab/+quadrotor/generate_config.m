%% Quadrotor configuration generator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function config = generate_config()

% drone config: drone parameters
config.mass = 0.3; % [kg]
config.inertia = diag([0.0007 0.0007 0.0015]); % [kg.m^2]
config.arms_length = 0.1; % [m]
config.inertia_inv = inv(config.inertia);

% drone config: motors parameters
rpm_to_rad_sec = pi/30;
config.propeller_lift_coeff = 9e-07; % [N/rad/s]
config.propeller_drag_coeff = 6e-09; % [N.m/rad/s]
config.propeller_speed_min = 3000 * rpm_to_rad_sec; % [rad/s]
config.propeller_speed_max = 20000 * rpm_to_rad_sec; % [rad/s]

% mix matrix
l = config.arms_length;
a = config.propeller_lift_coeff;
b = config.propeller_drag_coeff;
config.propeller_speed_min_square = config.propeller_speed_min^2;
config.propeller_speed_max_square = config.propeller_speed_max^2;
config.demix_matrix = [a    a    a    a
                             a*l -a*l -a*l  a*l
                             a*l  a*l -a*l -a*l
                             b   -b    b   -b];
config.mix_matrix = inv(config.demix_matrix);

% drone config: physics parameters
config.gravity = 9.81; % [m/s^2]

% drone config: attitude controller
config.attitude_gain =  0.05; % [N.m/rad]
config.angular_velocity_gain = 0.007;  % [N.m.s/rad]
config.angular_velocity_max = deg2rad(400);  % [rad/s]
end