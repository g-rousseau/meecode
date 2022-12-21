%% Draw function: 3D axes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function axes_handle = axes(attitude, origin, color, scale)

x = attitude * [1; 0; 0];
y = attitude * [0; 1; 0];
z = attitude * [0; 0; 1];

% World axes
hold on;
axes_handle(1) = plot3([0 x(1)]*scale + origin(1), ...
                       [0 x(2)]*scale + origin(2), ...
                       [0 x(3)]*scale + origin(3), ...
                       'color', color, ...
                       'Linewidth', 1.5);
axes_handle(2) = plot3([0 y(1)]*scale + origin(1), ...
                       [0 y(2)]*scale + origin(2), ...
                       [0 y(3)]*scale + origin(3), ...
                       'color', color, ...
                       'Linewidth', 1.5);
axes_handle(3) = plot3([0 z(1)]*scale + origin(1), ...
                       [0 z(2)]*scale + origin(2), ...
                       [0 z(3)]*scale + origin(3), ...
                       'color', color, ...
                       'Linewidth', 1.5);
end