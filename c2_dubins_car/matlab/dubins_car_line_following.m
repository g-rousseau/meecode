%% Dubin's car line following
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

%% Initialize simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial state
init_state.position = (rand(2,1) - 0.5) * 20; % [m]
init_state.angle = rand() * 2 * pi; % [m]

% Dubin's car config
dubins_config.speed_max = 2; % [m/s]
dubins_config.angular_speed_max = deg2rad(15); % [rad/s]

% simulation config
simulation_duration = 30; % [s]
simulation_step = 0.1; % [s]

% reference config
reference_config.line_origin = [0; 0];
reference_config.line_vector = [1; 0];

%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
state.position = init_state.position;
state.angle = init_state.angle;
time = 0;

trajectory.position = zeros(2, ceil(simulation_duration / simulation_step));
trajectory.position(:,1) = init_state.position;
trajectory.k = 1;

display_config = display_init(init_state, reference_config);
display_config = display_state(time, state, trajectory, display_config);

while time <= simulation_duration
    % control signal
    line_error = -state.position(2);
    
    angle_gain = 1;
    angle_ref = atan(angle_gain * line_error);
    
    angular_velocity_gain = 1;    
    command.forward_velocity = 1; % [m/s]
    command.angular_velocity = angular_velocity_gain * wrapToPi(angle_ref - state.angle); % [rad/s]
    
    % simulate car
    [state, command_sat] = dubins_car.simulate(state, command, simulation_step, dubins_config);
    
    % update display
    trajectory.k = trajectory.k + 1;
    trajectory.position(:, trajectory.k) = state.position;
    display_config = display_state(time, state, trajectory, display_config);
    
    % update time
    time = time + simulation_step;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_config = display_init(init_state, reference_config)
    display_config.map_bounds = 10; % [m]
    display_config.dead_zone = 0.4; % []
    display_config.car_scale = 1;
    display_config.car_color = [215, 42, 42] / 255;

    display_config.figure_handle = figure;
    display_config.origin = init_state.position;
    display_config.axes_handle = axes;
    
    figure(display_config.figure_handle);
    clf;
    hold on;
    
    % dead-zone
    display_config.origin = init_state.position;
    
    % reference line
    line_x = reference_config.line_origin(1)...
        + [-reference_config.line_vector(1), reference_config.line_vector(1)] * 10000;
    line_y = reference_config.line_origin(2)...
        + [-reference_config.line_vector(2), reference_config.line_vector(2)] * 10000;
    display_config.line_handle = plot(line_x, line_y, 'linewidth', 2);
    
    % trajectory
    trajectory_color = [255 119 119] / 255;
    display_config.trajectory_handle = plot(init_state.position(1), init_state.position(1),...
                                            'linewidth', 1.5,...
                                            'color', trajectory_color);
    
    % car
    display_config.car_handle = dubins_car.draw(init_state,...
                                                display_config.car_scale,...
                                                display_config.car_color);
    
    % prettify
    grid on;
    daspect([1 1 1]);
    xlim([-display_config.map_bounds, display_config.map_bounds] + display_config.origin(1));
    ylim([-display_config.map_bounds, display_config.map_bounds] + display_config.origin(2));
    xlabel('$x$ (m)', 'Interpreter', 'latex');
    ylabel('$y$ (m)', 'Interpreter', 'latex');
    title('\textbf{Dubins car}', ...
        'Interpreter', 'latex');
    drawnow;
end

function display_config = display_state(time, state, trajectory, display_config)
     
    % dead-zone
    dead_zone_start = display_config.dead_zone * display_config.map_bounds;
    delta = state.position - display_config.origin;
    if abs(delta(1)) > dead_zone_start
        ratio = dead_zone_start / abs(delta(1));
        delta = ratio * delta;
    end
    if abs(delta(2)) > dead_zone_start
        ratio = dead_zone_start / abs(delta(2));
        delta = ratio * delta;
    end
    display_config.origin = state.position - delta;
    
    % trajectory
    k = trajectory.k;
    display_config.trajectory_handle.XData = trajectory.position(1, 1:k);
    display_config.trajectory_handle.YData = trajectory.position(2, 1:k);
    
    % Dubin's car
    delete(display_config.car_handle);
    display_config.car_handle = dubins_car.draw(state,...
                                                display_config.car_scale,...
                                                display_config.car_color);

    % prettify
    xlim([-display_config.map_bounds, display_config.map_bounds] + display_config.origin(1));
    ylim([-display_config.map_bounds, display_config.map_bounds] + display_config.origin(2));
    title(['\textbf{Dubins car at} $t=', num2str(round(time)), '\mathrm{s}$'], ...
        'Interpreter', 'latex');
    drawnow;
end