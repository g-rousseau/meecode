%% Quadrotor simulation display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1: Initialize simulation by running init.m
% 2: Run simulink simulation
% 3: Run dispay_results.m

%% Attitude
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
clf;

time = out.euler_angles_ref.Time;
roll_ref = out.euler_angles_ref.Data(:,1);
pitch_ref = out.euler_angles_ref.Data(:,2);
yaw_ref = unwrap(out.euler_angles_ref.Data(:,3));
roll = out.euler_angles.Data(:,1);
pitch = out.euler_angles.Data(:,2);
yaw = unwrap(out.euler_angles.Data(:,3));

subplot(3,1,1);
plot(time, rad2deg(roll_ref));
hold on; grid on;
plot(time, rad2deg(roll));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\phi$ (degree)', 'Interpreter', 'latex');
title('\textbf{Roll tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,1,2);
plot(time, rad2deg(pitch_ref));
hold on; grid on;
plot(time, rad2deg(pitch));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\theta$ (degree)', 'Interpreter', 'latex');
title('\textbf{Pitch tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,1,3);
plot(time, rad2deg(yaw_ref));
hold on; grid on;
plot(time, rad2deg(yaw));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\psi$ (degree)', 'Interpreter', 'latex');
title('\textbf{Yaw tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Angular speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2)
clf;

time = out.euler_angles_ref.Time;
p_ref = out.angular_velocity_body_ref.Data(:,1);
q_ref = out.angular_velocity_body_ref.Data(:,2);
r_ref = out.angular_velocity_body_ref.Data(:,3);
p = out.angular_velocity_body.Data(:,1);
q = out.angular_velocity_body.Data(:,2);
r = out.angular_velocity_body.Data(:,3);

subplot(3,1,1);
plot(time, rad2deg(p_ref));
hold on; grid on;
plot(time, rad2deg(p));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$p$ (degree/s)', 'Interpreter', 'latex');
title('$x$\textbf{-axis angular velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,1,2);
plot(time, rad2deg(q_ref));
hold on; grid on;
plot(time, rad2deg(q));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$q$ (degree/s)', 'Interpreter', 'latex');
title('$y$\textbf{-axis angular velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,1,3);
plot(time, rad2deg(r_ref));
hold on; grid on;
plot(time, rad2deg(r));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$r$ (degree/s)', 'Interpreter', 'latex');
title('$z$\textbf{-axis angular velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Propellers speeds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w1 = out.propellers_speed.Data(:,1);
w2 = out.propellers_speed.Data(:,2);
w3 = out.propellers_speed.Data(:,3);
w4 = out.propellers_speed.Data(:,4);

rad_sec_to_rpm = 30/pi;

figure(3); clf;
plot(out.propellers_speed.Time, w1 * rad_sec_to_rpm);
hold on;
plot(out.propellers_speed.Time, w2 * rad_sec_to_rpm);
plot(out.propellers_speed.Time, w3 * rad_sec_to_rpm);
plot(out.propellers_speed.Time, w4 * rad_sec_to_rpm);
grid on;
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\omega_i$ (RPM)', 'Interpreter', 'latex');
title('\textbf{Propellers speeds}', 'Interpreter', 'latex');
legend({'motor 1', 'motor 2', 'motor 3', 'motor 4'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Quadrotor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = out.rotation_matrix.time;
R = out.rotation_matrix.Data;
R_ref = out.rotation_matrix_ref.Data;

figure(4);
clf;

dtplot = 0.1;
N = length(time);
dk = floor(dtplot / simulation_step);
quadScale = 1;
displayAxes = 1;
axesScale = 3;
for i = 1:dk:N
    figure(4);
    clf;
    
    % Reference (converted in ENU - more intuitive)
    R_ned_to_enu = [1 0 0;0 -1 0; 0 0 -1];
    R_enu = R_ned_to_enu * R(:,:,i);
    R_enu_ref = R_ned_to_enu * R_ref(:,:,i);
    
    draw.axes(R_ned_to_enu, zeros(3,1), 'k', axesScale);
    draw.axes(R_enu_ref, zeros(3,1), 'r', axesScale);
    draw.axes(R_enu, zeros(3,1), 'g', axesScale);
    
    % Drone    
    quadrotor.draw(R_enu, zeros(3,1), quadScale, 'b');
    xlabel('x', 'Interpreter', 'latex');
    ylabel('y', 'Interpreter', 'latex');
    zlabel('z', 'Interpreter', 'latex');
    daspect([1 1 1]);
    set(gca, 'view', [117 28]);
    grid on;
    xlim([-1 1]*axesScale);
    ylim([-1 1]*axesScale);
    zlim([-1 1]*axesScale);
    
    title_fig = ['\textbf{Quadrotor pose at time} $t=', num2str(round(time(i))), '\mathrm{s}$'];
    title(title_fig, 'Interpreter', 'latex');
    drawnow;
end