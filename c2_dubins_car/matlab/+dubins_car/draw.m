%% Dubins car display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dubins_handle = draw(state, scale, color)
% state: current state of the Ddubin's car
%         state.position (2x1 vector) [m]
%         state.angle [rad]
% scale: car scale (1 for a car fitting in a 2mx2m square)
% color: car color
%
% dubins_handle: axes handle of the drawing
%

c = cos(state.angle); s = sin(state.angle);
R = [c, -s;
     s,  c];

% body
body = [1,  0.7, -0.9, -0.9,  0.7, 1;
        0,  0.6,  0.6, -0.6, -0.6, 0];
body = R * body * scale + state.position;

% wheels
wheel_color = [15, 15, 15] / 255;
wheel_1 = [0.75, 0.75, -0.75, -0.75, 0.75;
           0.7,  0.95,  0.95,  0.7, 0.7];
wheel_2 = [ 0.75, 0.75, -0.75, -0.75, 0.75;
           -0.7,  -0.95, -0.95,  -0.7, -0.7];
wheel_1 = R * wheel_1 * scale + state.position;
wheel_2 = R * wheel_2 * scale + state.position;

% axle
axle_color = [160, 160, 160] / 255;
axle = [0.1, -0.1, -0.1,  0.1, 0.1;
        0.9,  0.9, -0.9, -0.9, 0.9];
axle = R * axle * scale + state.position;

hold on;
dubins_handle(1) = fill(axle(1,:), axle(2,:), axle_color, 'EdgeAlpha', 0);
dubins_handle(2) = fill(body(1,:), body(2,:), color, 'EdgeAlpha', 0);
dubins_handle(3) = fill(wheel_1(1,:), wheel_1(2,:), wheel_color, 'EdgeAlpha', 0);
dubins_handle(4) = fill(wheel_2(1,:), wheel_2(2,:), wheel_color, 'EdgeAlpha', 0);
end