%% Dubins car model propagation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [state_next, command_sat] = simulate(state, command, timestep, config)
% propagate the model of a Dubin's car
% state: current state of the Ddubin's car
%         state.position (2x1 vector) [m]
%         state.angle [rad]
% command: control inputs
%          command.forward_velocity [m/s]
%          command.angular_velocity [rad/s]
% timestep: simulation timestep [s]
% config: Dubin's car configuration
%         config.speed_max [m/s]
%         config.angular_speed_max [rad/s]
%
% state_next: propagated state (state at next timestep)
%             state_next.position (2x1 vector) [m]
%             state_next.angle [rad]
% command_sat: saturated control inputs
%              command_sat.forward_velocity [m/s]
%              command_sat.angular_velocity [rad/s]
%

% command saturation
command_sat.forward_velocity = ...
    saturate(command.forward_velocity, -config.speed_max, config.speed_max);
command_sat.angular_velocity = ...
    saturate(command.angular_velocity, -config.angular_speed_max, config.angular_speed_max);

% propagate model
state_next.angle = state.angle + command.angular_velocity * timestep;
velocity = [command.forward_velocity * cos(state.angle);
            command.forward_velocity * sin(state.angle)];
state_next.position = state.position + velocity * timestep;
end

function u_sat = saturate(u, u_min, u_max)
    u_sat = min([u_max max([u_min u])]);
end