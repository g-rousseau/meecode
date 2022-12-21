%% Bezier curve evaluation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function values = evaluate(control_points, tau)
% evaluate a bezier curve of dimension d, containing n control points, at tau in [0, 1]
% control_points: control points (d x n)
% tau: values at which to evaluate the curve, in [0, 1] (1 x N)
%
% value: evaluation of the curve at tau (d x N)
%

sample_cnt = length(tau);
dimension = size(control_points, 1);
values = zeros(dimension, sample_cnt);
n = size(control_points, 2) - 1;

for i_sample = 1:sample_cnt
    t = tau(i_sample);
    tmp = control_points;
    for i = 1:n
        for j = 1:(n+1-i)
            tmp(:, j) = (1-t) * tmp(:, j) + t * tmp(:, j+1);
        end
    end
    values(:, i_sample) = tmp(:, 1);
end
end