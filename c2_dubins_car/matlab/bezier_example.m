%% Bezier curve example
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

control_point = [1 2  3 4;
                 0 1 -1 0];
sample_cnt = 100;

tau = 0:(1/(sample_cnt-1)):1;
curve = bezier.evaluate(control_point, tau);

fig_bezier = figure(1); clf;
hold on;

plot(curve(1,:), curve(2,:), 'linewidth', 1.25);
plot(control_point(1,:), control_point(2,:), 'k:', 'linewidth', 0.75);
plot(control_point(1,:), control_point(2,:), 'k+', 'linewidth', 2);

grid on;
daspect([1 1 1]);
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
title('\textbf{Bezier curve}', 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex')