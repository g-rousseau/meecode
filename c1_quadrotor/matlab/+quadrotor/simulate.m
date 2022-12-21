%% Quadrotor model propagation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [state_next, command] = simulate(state, command, timestep, config)

% saturate command
command = mix(command, config);

% state derivative
[acceleration_ned, quaternion_derivative, angular_acceleration_body] = ...
    propagate_model(command, state, config);

% integrate state derivative
state_next.angular_velocity_body = ...
    state.angular_velocity_body + angular_acceleration_body * timestep;

state_next.quaternion = state.quaternion + quaternion_derivative * timestep;
state_next.quaternion = state_next.quaternion / norm(state_next.quaternion);
state_next.euler_angles = utils.quaternion_to_euler_angles(state_next.quaternion);
state_next.rotation_matrix = utils.quaternion_to_rotation_matrix(state_next.quaternion);

state_next.velocity_ned = state.velocity_ned + acceleration_ned * timestep;
state_next.position_ned = state.position_ned + state_next.velocity_ned * timestep;
end

%% Mix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function command = mix(command, config)

propellers_speed_ref_square = config.mix_matrix * [command.thrust_ref; command.torque_ref];

command.propellers_speed = zeros(4,1);
for i = 1:4
    swi = propellers_speed_ref_square(i);
    swi = min([config.propeller_speed_max_square, max([config.propeller_speed_min_square swi])]);
    command.propellers_speed(i) = sqrt(swi);
end

command.thrust = config.demix_matrix(1,:) * command.propellers_speed.^2;
command.torques = config.demix_matrix(2:4,:) * command.propellers_speed.^2;
end

%% Quadrotor dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [acceleration_ned, ...
          quaternion_derivative, ...
          angular_acceleration_body] = propagate_model(command, state, config)


acceleration_ned = thrust_vector_ned(command.thrust, state.quaternion) / config.mass;
acceleration_ned(3) = acceleration_ned(3) + config.gravity;

quaternion_derivative = 0.5 * utils.quaternion_to_matrix(state.quaternion)...
                        * utils.vector_to_quaternion(state.angular_velocity_body);

gyro_torque = -cross(state.angular_velocity_body, config.inertia * state.angular_velocity_body);
angular_acceleration_body = config.inertia_inv * (command.torques + gyro_torque);
end

%% Thrust and attitude to thrust vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function thrust_ned = thrust_vector_ned(f,q)
    thrust_ned = -f * [2 * (q(1)*q(3) + q(2)*q(4))
                       2 * (q(3)*q(4) - q(1)*q(2))
                       1 - 2 * (q(2)^2 + q(3)^1)];
end
