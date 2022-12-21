%% Quadrotor acceleration control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [command, command_prev] = ...
    control_acceleration(reference, state, command_prev, timestep, config)

% trhust reference
thrust_vector = config.mass * (reference.acceleration_ned - [0; 0; config.gravity]);
thrust_vector_norm = norm(thrust_vector);
if thrust_vector_norm > 10 * eps
    z_body_ref = -thrust_vector / thrust_vector_norm;
else
    z_body_ref = [0; 0; 1];
end
z_body = state.rotation_matrix(:,3);
command.thrust_ref = thrust_vector_norm * z_body_ref'* z_body;

% attitude reference
R_ref = z_and_yaw_to_rotation_matrix(z_body_ref, reference.yaw);
d_R_ref = (R_ref - command_prev.rotation_matrix_ref) / timestep;
w_ref = utils.saturate_norm(utils.vee(R_ref' * d_R_ref), config.angular_velocity_max);
d_w_ref = (w_ref - command_prev.angular_velocity_body_ref) / timestep;

% attitude control
R = state.rotation_matrix;
w = state.angular_velocity_body;
w_hat = utils.hat(w);
J = config.inertia;

attitude_error = -0.5 * utils.vee(R_ref' * R - R' * R_ref);
angular_velocity_error =  R' * R_ref * w_ref - w;
gyro_torque_compensation = w_hat * J * w;
torque_feedforward = J * (R' * R_ref * d_w_ref - w_hat * R' * R_ref * w_ref);
command.torque_ref = config.attitude_gain * attitude_error ...
                     + config.angular_velocity_gain * angular_velocity_error ...
                     + gyro_torque_compensation ...
                     + torque_feedforward;

command.angular_velocity_body_ref = w_ref;
command.rotation_matrix_ref = R_ref;
command.euler_angles_ref = utils.rotation_matrix_to_euler_angles(R_ref)';

command_prev.rotation_matrix_ref = command.rotation_matrix_ref;
command_prev.angular_velocity_body_ref = command.angular_velocity_body_ref;
end

%% Z axis and heading to rotation matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rotation_matrix = z_and_yaw_to_rotation_matrix(z, yaw)

yaw_vect = [cos(yaw); sin(yaw); 0];
y = cross(z, yaw_vect);
y = y / norm(y); % singularity if x and z are collinear
x = cross(y, z);

rotation_matrix = [x, y, z];
end