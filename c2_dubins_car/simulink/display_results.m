%% Dubins car simulation display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 1.3: Dubins car modeling
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1: Initialize simulation by running init.m
% 2: Run either simulation.slx or simulation_no_simulink.m
% 3: Run display_results.m

%% Position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2)
clf;

time = out.position_ref.Time;
x_ref = out.position_ref.Data(:,1);
y_ref = out.position_ref.Data(:,2);
x = out.position.Data(:,1);
y = out.position.Data(:,2);

subplot(2,2,1);
plot(time, x_ref);
hold on; grid on;
plot(time, x);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$x$ (m)', 'Interpreter', 'latex');
title('$x$\textbf{-axis position tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(2,2,3);
plot(time, y_ref);
hold on; grid on;
plot(time, y);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$y$ (m)', 'Interpreter', 'latex');
title('$y$\textbf{-axis position tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

time = out.velocity_ref.Time;
vx_ref = out.velocity_ref.Data(:,1);
vy_ref = out.velocity_ref.Data(:,2);
vx = out.velocity.Data(:,1);
vy = out.velocity.Data(:,2);

subplot(2,2,2);
plot(time, vx_ref);
hold on; grid on;
plot(time, vx);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\dot{x}$ (m/s)', 'Interpreter', 'latex');
title('$x$\textbf{-axis velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(2,2,4);
plot(time, vy_ref);
hold on; grid on;
plot(time, vy);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'latex');
title('$y$\textbf{-axis velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(3)
clf;

time = out.forward_velocity.Time;
v_ref = out.forward_velocity_ref.Data(:,1);
w_ref = out.angular_velocity_ref.Data(:,1);
v = out.forward_velocity.Data(:,1);
w = out.angular_velocity.Data(:,1);

subplot(2,1,1);
plot(time, v_ref);
hold on; grid on;
plot(time, v);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$v$ (m/s)', 'Interpreter', 'latex');
title('\textbf{Forward velocity}', 'Interpreter', 'latex');
legend({'Reference', 'Saturated'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(2,1,2);
plot(time, rad2deg(w_ref));
hold on; grid on;
plot(time, rad2deg(w));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\omega$ (degree/s)', 'Interpreter', 'latex');
title('\textbf{Angular velocity}', 'Interpreter', 'latex');
legend({'Reference', 'Saturated'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(4)
clf;

time = out.angle.Time;
theta = out.angle.Data(:,1);

plot(time, theta);
hold on; grid on;
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\theta$ (degree)', 'Interpreter', 'latex');
title('\textbf{Angle}', 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Animation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(5)
clf;

time = out.angle.Time;
theta = out.angle.Data(:,1);

N = length(time);
dN = 1;
i_span = 1:dN:N;
Ni = length(i_span);
origin = [x(1); y(1)];
for i = i_span
    figure(5);
    clf;
    hold on;
    plot(x_ref(1:i), y_ref(1:i), 'r', 'Linewidth', 1.5);
    plot(x_ref(i), y_ref(i), 'ro', 'Linewidth', 2.5);
    plot(x(1:i), y(1:i), 'b', 'Linewidth', 1.5);
    origin = draw_functions.draw_dubins([x(i);y(i)], theta(i), origin, map_bounds, dead_zone);
    
    grid on;
    daspect([1 1 1]);
    xlabel('$x$ (m)', 'Interpreter', 'latex');
    ylabel('$y$ (m)', 'Interpreter', 'latex');
    title(['\textbf{Dubins car position at} $t=', num2str(round(time(i))), '\mathrm{s}$'], ...
        'Interpreter', 'latex');
    drawnow;
end