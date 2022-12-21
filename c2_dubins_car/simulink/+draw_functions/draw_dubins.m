%% Dubins car display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function origin = display_dubins(position, angle, origin_prev, map_bounds, dead_zone)

x_car = [0.7 -1  -1    0.7 1.3 0.7 -1   -1    0.7 0.7];
y_car = [0.7 0.7 -0.7 -0.7 0   0.7 -0.7  0.7 -0.7 0.7];

c = cos(angle); s = sin(angle);
x_draw = c * x_car - s * y_car;
y_draw = s * x_car + c * y_car;

x_draw = x_draw + position(1);
y_draw = y_draw + position(2);

dead_zone_start = dead_zone * map_bounds;
delta = position - origin_prev;
if abs(delta(1)) > dead_zone_start
    ratio = dead_zone_start / abs(delta(1));
    delta = ratio * delta;
end
if abs(delta(2)) > dead_zone_start
    ratio = dead_zone_start / abs(delta(2));
    delta = ratio * delta;
end
origin = position - delta;

% figure(2);
plot(x_draw, y_draw, 'k', 'linewidth', 1.5);
xlim([-map_bounds map_bounds]+origin(1));
ylim([-map_bounds map_bounds]+origin(2));
end