%% Dubins car simulation intialization
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

step_cnt = ceil(simulation_duration / simulation_step) + 1;

time = 0:(step_cnt-1);
time = time * simulation_step;

position_ref = zeros(step_cnt, 2);
velocity_ref = zeros(step_cnt, 2);
position = zeros(step_cnt, 2);
velocity = zeros(step_cnt, 2);
angle_ref = zeros(step_cnt, 1);
angle = zeros(step_cnt, 1);
forward_velocity_ref = zeros(step_cnt, 1);
forward_velocity = zeros(step_cnt, 1);
angular_velocity_ref = zeros(step_cnt, 1);
angular_velocity = zeros(step_cnt, 1);

position(1,:) = init_position';
angle(1) = init_angle;

wx = x_ref_frequency * 2*pi;
wy = y_ref_frequency * 2*pi;
for i = 1:step_cnt
    t = time(i);
    
    % reference
    position_ref(i,:) = [x_ref_mag * sin(wx * t), ...
                         y_ref_mag * sin(wy * t + y_ref_phase)];
    velocity_ref(i,:) = [x_ref_mag * wx * cos(wx * t), ...
                         y_ref_mag * wy * cos(wy * t + y_ref_phase)];
    
    % control - position
    angle_ref(i) = ;
    forward_velocity_ref(i) = ;
    
    % control - angle
    angular_velocity_ref(i) = ;
        
    % model
    forward_velocity(i) = saturate(forward_velocity_ref(i), -speed_max, speed_max);
    angular_velocity(i) = saturate(angular_velocity_ref(i), -angular_speed_max, angular_speed_max);
    if i < step_cnt
        angle(i+1,:) = angle(i,:) + angular_velocity(i) * simulation_step;
        velocity(i+1,:) = [forward_velocity(i) * cos(angle(i,:)), ...
                           forward_velocity(i) * sin(angle(i,:))];
        position(i+1,:) = position(i,:) + velocity(i,:) * simulation_step;
    end
end

out.position_ref.Time = time;
out.position_ref.Data = position_ref;
out.position.Time = time;
out.position.Data = position;
out.velocity_ref.Time = time;
out.velocity_ref.Data = velocity_ref;
out.velocity.Time = time;
out.velocity.Data = velocity;
out.angle_ref.Time = time;
out.angle_ref.Data = angle_ref;
out.angle.Time = time;
out.angle.Data = angle;
out.forward_velocity_ref.Time = time;
out.forward_velocity_ref.Data = forward_velocity_ref;
out.forward_velocity.Time = time;
out.forward_velocity.Data = forward_velocity;
out.angular_velocity_ref.Time = time;
out.angular_velocity_ref.Data = angular_velocity_ref;
out.angular_velocity.Time = time;
out.angular_velocity.Data = angular_velocity;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function u_sat = saturate(u, u_min, u_max)
    u_sat = min([u_max max([u_min u])]);
end