%% Dubin's car position control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

%% Initialize simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initial state
init_state.position = (rand(2,1) - 0.5) * 1; % [m]
init_state.angle = rand() * 2 * pi; % [m]

init_state.position = [10;0];
init_state.angle = deg2rad(20); % [m]

% Dubin's car config
dubins_config.speed_max = 2; % [m/s]
dubins_config.angular_speed_max = deg2rad(30); % [rad/s]
dubins_config.angle_gain = 1;

% simulation config
simulation_duration = 30; % [s]
simulation_step = 0.1; % [s]

% reference config
control_points = [1 2  3 4;
                  0 1 -1 0]*10;

%% Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
state.position = init_state.position;
state.angle = init_state.angle;
time = 0;

trajectory.position = zeros(2, ceil(simulation_duration / simulation_step));
trajectory.position(:,1) = init_state.position;
trajectory.position_reference = zeros(2, ceil(simulation_duration / simulation_step));
trajectory.position_reference(:,1) = control_points(:,1);
trajectory.k = 1;

display_config = display_init(init_state, trajectory, control_points);
display_config = display_state(time, state, trajectory, display_config);

while time <= simulation_duration
    % reference
    position_ref = bezier.evaluate_trajectory(control_points, time, 0, simulation_duration);
    velocity_ref = bezier.evaluate_trajectory(control_points, time, 1, simulation_duration);
                
    % position control signal
    position_gain = 1;
    reference_velocity = position_gain * (position_ref - state.position) + velocity_ref;

    % velocity control signal
    command = dubins_car.control_velocity(reference_velocity, state, dubins_config);
    
    % simulate car
    [state, command_sat] = dubins_car.simulate(state, command, simulation_step, dubins_config);
    
    % update display
    trajectory.k = trajectory.k + 1;
    trajectory.position(:, trajectory.k) = state.position;
    trajectory.position_reference(:,trajectory.k) = position_ref;
    display_config = display_state(time, state, trajectory, display_config);
    
    % update time
    time = time + simulation_step;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOCAL FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function display_config = display_init(init_state, trajectory, control_point)
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
    
    % bezier curve
    tau = 0:0.01:1;
    curve = bezier.evaluate(control_point, tau);
    display_config.bezier_curve = plot(curve(1,:), curve(2,:), 'linewidth', 1.25);
    display_config.control_points = ...
        plot(control_point(1,:), control_point(2,:), 'k:', 'linewidth', 0.75);
    display_config.control_polygon = ...
        plot(control_point(1,:), control_point(2,:), 'k+', 'linewidth', 2);
    
    % reference trajectory
    reference_trajectory_color = [255 119 119] / 255;
    display_config.reference_handle = ...
        plot(trajectory.position_reference(1), trajectory.position_reference(1),...
             'linewidth', 1.5,...
             'color', reference_trajectory_color);
    
    % trajectory
    trajectory_color = [119 119 255] / 255;
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
    
    % reference trajectory
    k = trajectory.k;
    display_config.reference_handle.XData = trajectory.position_reference(1, 1:k);
    display_config.reference_handle.YData = trajectory.position_reference(2, 1:k);
    
    % trajectory
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