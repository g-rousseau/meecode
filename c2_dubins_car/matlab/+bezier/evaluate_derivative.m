%% Bezier curve derivative evaluation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function values = evaluate_derivative(control_points, tau, derivative_order)
% evaluate the derivative of a bezier curve of dimension d, containing n control points,
% at tau in [0, 1]
% control_points: control points (d x n)
% tau: values at which to evaluate the curve, in [0, 1] (1 x N)
% derivative_order: derivative order (>= 0)
%
% value: evaluation of the derivative of the curve at tau (d x N)
%

if derivative_order < 0
    error('derivative order must be positive');
end

n = size(control_points, 2) - 1;

if derivative_order > n
    sample_cnt = length(tau);
    dimension = size(control_points, 1);
    values = zeros(dimension, sample_cnt);
    return;
end

tmp = control_points;
for i = 1:derivative_order
    for j = 1:(n+1-i)
        tmp(:, j) = n * (tmp(:, j+1) - tmp(:, j));
    end
end
derivative_control_points = tmp(:, 1:(n+1-derivative_order));
values = bezier.evaluate(derivative_control_points, tau);
end