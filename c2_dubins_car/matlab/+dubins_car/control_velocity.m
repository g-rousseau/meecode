%% Dubins car velocity control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [command, angle_ref] = control_velocity(reference_velocity, state, config)
% compute the control inputs for velocity control of a Dubin's 
% reference_velocity: 
% state: current state of the Ddubin's car
%        state.position (2x1 vector) [m]
%        state.angle [rad]
% config: Dubin's car configuration
%         config.speed_max [m/s]
%         config.angular_speed_max [rad/s]
%         config.angle_gain [/s]
%
% command: control inputs
%          command.forward_velocity [m/s]
%          command.angular_velocity [rad/s]
%

% forward velocity command: velocity command projected on direction
direction = [cos(state.angle), sin(state.angle)];
command.forward_velocity = max([0, direction * reference_velocity]);

% angle control: proportional control on angle
angle_ref = atan2(reference_velocity(2), reference_velocity(1));
command.angular_velocity = config.angle_gain * wrapToPi(angle_ref - state.angle);
end