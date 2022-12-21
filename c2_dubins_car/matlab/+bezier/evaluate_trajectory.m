%% Bezier trajectory evaluation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function values = evaluate_trajectory(control_points, time, derivative_order, duration)
% evaluate a derivative of a bezier curve trajectory of dimension d, containing n control points,
% and of duration T, at time t in [0, T]
% control_points: control points (d x n)
% time: times at which to evaluate the trajectory, in [0, T] (1 x N)
% derivative_order: derivative order (>= 0)
% duration: duration of the trajectory (> 0) [s]
%
% value: evaluation of the derivative of the curve at tau (d x N)
%

if derivative_order < 0
    error('derivative order must be positive');
end
if duration <= 0
    error('duration must be strictly positive');
end

tau = time / duration;
derivative_factor = 1 / duration^derivative_order;
values = derivative_factor * bezier.evaluate_derivative(control_points, tau, derivative_order);
end