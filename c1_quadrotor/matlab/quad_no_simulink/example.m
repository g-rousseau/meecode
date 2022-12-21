%% Example: position control of an acceleration-controlled quadrotor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

% Initialize simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% drone config
drone_config = quadrotor.generate_config();

% initial state
init_state = generate_initial_state();

% reference config
reference_config = generate_reference_config(drone_config);

% display config
display_config = generate_display_config();

% simulation config
simulation_step = 0.005; % [s]
simulation_duration = 20; % [s]

% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
command_prev.rotation_matrix_ref = init_state.rotation_matrix;
command_prev.angular_velocity_body_ref = init_state.angular_velocity_body;
state = init_state;

time = 0;
reference = acceleration_reference(time, state, reference_config);
display_config = display_init(time, state, reference, display_config);

i_disp = 0;
disp_cnt = floor(display_config.display_rate / simulation_step);
while time <= simulation_duration
    % update reference
    reference = acceleration_reference(time, state, reference_config);
    
    % update velocity acceleration-controlled quadrotor state
    [command, command_prev] = ...
        quadrotor.control_acceleration(reference, state, command_prev, simulation_step, drone_config);
    [state, command] = quadrotor.simulate(state, command, simulation_step, drone_config);
    
    % update display
    if i_disp > disp_cnt
        i_disp = 0;
        display_config = display_state(time, state, reference, display_config);
    else
        i_disp = i_disp + 1;
    end
    
    % update time
    time = time + simulation_step;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function initial_state = generate_initial_state()
    initial_state.position_ned = [0; 2; 0]; % [m]
    initial_state.velocity_ned = [0; 0; 0]; % [m/s]
    initial_state.euler_angles.yaw = 0; % [rad]
    initial_state.euler_angles.pitch = 0; % [rad]
    initial_state.euler_angles.roll = 0; % [rad]
    initial_state.quaternion = utils.euler_angles_to_quaternion(initial_state.euler_angles);
    initial_state.rotation_matrix = utils.quaternion_to_rotation_matrix(initial_state.quaternion);
    initial_state.angular_velocity_body = [0; 0; 0]; % [rad/s]
end

% Reference config
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reference_config = generate_reference_config(drone_config)
    reference_config.x_magnitude = 2; % [m]
    reference_config.y_magnitude = 2; % [m]
    reference_config.z_magnitude = 2; % [m]
    reference_config.yaw_magnitude = deg2rad(360); % [rad]

    reference_config.x_frequency = 0.1; % [Hz]
    reference_config.y_frequency = 0.1; % [Hz]
    reference_config.z_frequency = 0.1; % [Hz]
    reference_config.yaw_frequency = 0.01; % [Hz]

    reference_config.y_phase = pi/2; % [rad]

    angle_max = deg2rad(15); % [rad]
    thrust_min = drone_config.propeller_lift_coeff * drone_config.propeller_speed_min^2;
    thrust_max = drone_config.propeller_lift_coeff * drone_config.propeller_speed_max^2;
    reference_config.position_gain = 3; % [/s^2]
    reference_config.velocity_gain = 3; % [/s]
    reference_config.acceleration_max = min([drone_config.gravity - thrust_min / drone_config.mass, ...
                                             thrust_max / drone_config.mass - drone_config.gravity, ...
                                             drone_config.gravity * sin(angle_max)]);
end

% Display config
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function display_config = generate_display_config()
    display_config.quadrotor_scale = 0.2;
    display_config.quadrotor_color = 'b';
    display_config.axes_scale = 1;
    display_config.ned_to_nwu = [1 0 0;0 -1 0; 0 0 -1];
    display_config.display_rate = 0.05;
end

% Reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reference = acceleration_reference(time, state, reference_config)
    x_puls = 2*pi * reference_config.x_frequency;
    y_puls = 2*pi * reference_config.y_frequency;
    z_puls = 2*pi * reference_config.z_frequency;
    yaw_puls = 2*pi * reference_config.yaw_frequency;
    
    % position reference
    reference.position_ned = ...
        [reference_config.x_magnitude * sin(x_puls * time);
         reference_config.y_magnitude * sin(y_puls * time + reference_config.y_phase);
         reference_config.z_magnitude * sin(z_puls * time)];
    reference.yaw = reference_config.yaw_magnitude * sin(yaw_puls * time);
    
    % velocity reference
    reference.velocity_ned = ...
        [reference_config.x_magnitude * x_puls * cos(x_puls * time);
         reference_config.y_magnitude * y_puls * cos(y_puls * time+ reference_config.y_phase)
         reference_config.z_magnitude * x_puls * cos(z_puls * time)];
    
    % acceleration feedforward
    reference.acceleration_ned = ...
        [-reference_config.x_magnitude * x_puls^2 * sin(x_puls * time);
         -reference_config.y_magnitude * y_puls^2 * sin(y_puls * time + reference_config.y_phase)
         -reference_config.z_magnitude * x_puls^2 * sin(z_puls * time)];
    
    % position and velocity feedbacks (proportional-derivative position control)
    reference.acceleration_ned = reference.acceleration_ned ...
        + reference_config.position_gain * (reference.position_ned - state.position_ned) ...
        + reference_config.velocity_gain * (reference.velocity_ned - state.velocity_ned);
    
    reference.acceleration_ned = ...
        utils.saturate_norm(reference.acceleration_ned, reference_config.acceleration_max);
end

% Display state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function display_config = display_init(time, state, reference, display_config)

    display_config.figure_handle = figure;
    clf;
    hold on;
    
    attitude_enu = display_config.ned_to_nwu * state.rotation_matrix;
    position_enu = display_config.ned_to_nwu * state.position_ned;
    position_ref_enu = display_config.ned_to_nwu * reference.position_ned;
    
    % NED frame
    display_config.ned_frame_handle = ...
        draw.axes(display_config.ned_to_nwu, position_enu, 'm', display_config.axes_scale);
    
    % Drone frame
    display_config.drone_frame_handle = ...
        draw.axes(attitude_enu, position_enu, 'c', display_config.axes_scale);
    
    % Drone
    display_config.drone_handle = quadrotor.draw(attitude_enu, ...
                                                 position_enu, ...
                                                 display_config.quadrotor_scale, ...
                                                 display_config.quadrotor_color);
    
    % reference
    display_config.reference_handle = ...
        plot3(position_ref_enu(1), position_ref_enu(2), position_ref_enu(3), 'ro', 'linewidth', 3);
    
    % prettify
    xlabel('$x$ (m)', 'Interpreter', 'latex');
    ylabel('$y$ (m)', 'Interpreter', 'latex');
    zlabel('$z$ (m)', 'Interpreter', 'latex');
    daspect([1 1 1]);
    set(gca, 'view', [117 28]);
    grid on;
    xlim([-1 1] * display_config.axes_scale + position_enu(1));
    ylim([-1 1] * display_config.axes_scale + position_enu(2));
    zlim([-1 1] * display_config.axes_scale + position_enu(3));
    
    title_fig = ['\textbf{Quadrotor at} $t=', num2str(round(time)), '\mathrm{s}$'];
    title(title_fig, 'Interpreter', 'latex');
    drawnow;
end

function display_config = display_state(time, state, reference, display_config)

    figure(display_config.figure_handle);
        
    attitude_enu = display_config.ned_to_nwu * state.rotation_matrix;
    position_enu = display_config.ned_to_nwu * state.position_ned;
    position_ref_enu = display_config.ned_to_nwu * reference.position_ned;
    
    % NED frame
    delete(display_config.ned_frame_handle);
    display_config.ned_frame_handle = ...
        draw.axes(display_config.ned_to_nwu, position_enu, 'm', display_config.axes_scale);
    
    % Drone frame
    delete(display_config.drone_frame_handle);
    display_config.drone_frame_handle = ...
        draw.axes(attitude_enu, position_enu, 'c', display_config.axes_scale);
    
    % Drone
    delete(display_config.drone_handle);
    display_config.drone_handle = quadrotor.draw(attitude_enu, ...
                                                 position_enu, ...
                                                 display_config.quadrotor_scale, ...
                                                 display_config.quadrotor_color);
    
    % reference
    delete(display_config.reference_handle);
    display_config.reference_handle = ...
        plot3(position_ref_enu(1), position_ref_enu(2), position_ref_enu(3), 'ro', 'linewidth', 3);
    
    xlim([-1 1] * display_config.axes_scale + position_enu(1));
    ylim([-1 1] * display_config.axes_scale + position_enu(2));
    zlim([-1 1] * display_config.axes_scale + position_enu(3));
    title_fig = ['\textbf{Quadrotor at} $t=', num2str(round(time)), '\mathrm{s}$'];
    title(title_fig, 'Interpreter', 'latex');
    
    drawnow;
end