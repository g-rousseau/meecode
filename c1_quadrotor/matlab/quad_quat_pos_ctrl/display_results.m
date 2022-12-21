%% Quadrotor simulation display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1: Initialize simulation by running init.m
% 2: Run simulink simulation
% 3: Run dispay_results.m

%% Position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
clf;

time = out.position_ned_ref.Time;
x_ref = out.position_ned_ref.Data(:,1);
y_ref = out.position_ned_ref.Data(:,2);
z_ref = out.position_ned_ref.Data(:,3);
x = out.position_ned.Data(:,1);
y = out.position_ned.Data(:,2);
z = out.position_ned.Data(:,3);

subplot(3,2,1);
plot(time, x_ref);
hold on; grid on;
plot(time, x);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$x$ (m)', 'Interpreter', 'latex');
title('$x$\textbf{-axis position tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,3);
plot(time, y_ref);
hold on; grid on;
plot(time, y);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$y$ (m)', 'Interpreter', 'latex');
title('$y$\textbf{-axis position tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,5);
plot(time, z_ref);
hold on; grid on;
plot(time, z);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$z$ (m)', 'Interpreter', 'latex');
title('$z$\textbf{-axis position tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

time = out.velocity_ned_ref.Time;
vx_ref = out.velocity_ned_ref.Data(:,1);
vy_ref = out.velocity_ned_ref.Data(:,2);
vz_ref = out.velocity_ned_ref.Data(:,3);
vx = out.velocity_ned.Data(:,1);
vy = out.velocity_ned.Data(:,2);
vz = out.velocity_ned.Data(:,3);

subplot(3,2,2);
plot(time, vx_ref);
hold on; grid on;
plot(time, vx);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\dot{x}$ (m/s)', 'Interpreter', 'latex');
title('$x$\textbf{-axis velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,4);
plot(time, vy_ref);
hold on; grid on;
plot(time, vy);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\dot{y}$ (m/s)', 'Interpreter', 'latex');
title('$y$\textbf{-axis velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,6);
plot(time, vz_ref);
hold on; grid on;
plot(time, vz);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\dot{z}$ (m/s)', 'Interpreter', 'latex');
title('$z$\textbf{-axis velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Attitude
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(2)
clf;

time = out.euler_angles_ref.Time;
roll_ref = out.euler_angles_ref.Data(:,1);
pitch_ref = out.euler_angles_ref.Data(:,2);
yaw_ref = unwrap(out.euler_angles_ref.Data(:,3));
roll = out.euler_angles.Data(:,1);
pitch = out.euler_angles.Data(:,2);
yaw = unwrap(out.euler_angles.Data(:,3));

subplot(3,2,1);
plot(time, rad2deg(roll_ref));
hold on; grid on;
plot(time, rad2deg(roll));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\phi$ (degree)', 'Interpreter', 'latex');
title('\textbf{Roll tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,3);
plot(time, rad2deg(pitch_ref));
hold on; grid on;
plot(time, rad2deg(pitch));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\theta$ (degree)', 'Interpreter', 'latex');
title('\textbf{Pitch tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,5);
plot(time, rad2deg(yaw_ref));
hold on; grid on;
plot(time, rad2deg(yaw));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$\psi$ (degree)', 'Interpreter', 'latex');
title('\textbf{Yaw tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

time = out.euler_angles_ref.Time;
p_ref = out.angular_velocity_body_ref.Data(:,1);
q_ref = out.angular_velocity_body_ref.Data(:,2);
r_ref = out.angular_velocity_body_ref.Data(:,3);
p = out.angular_velocity_body.Data(:,1);
q = out.angular_velocity_body.Data(:,2);
r = out.angular_velocity_body.Data(:,3);

subplot(3,2,2);
plot(time, rad2deg(p_ref));
hold on; grid on;
plot(time, rad2deg(p));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$p$ (degree/s)', 'Interpreter', 'latex');
title('$x$\textbf{-axis angular velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,4);
plot(time, rad2deg(q_ref));
hold on; grid on;
plot(time, rad2deg(q));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$q$ (degree/s)', 'Interpreter', 'latex');
title('$y$\textbf{-axis angular velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(3,2,6);
plot(time, rad2deg(r_ref));
hold on; grid on;
plot(time, rad2deg(r));
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$r$ (degree/s)', 'Interpreter', 'latex');
title('$z$\textbf{-axis angular velocity tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Quaternion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = out.quaternion_ref.Time;
q0_ref = out.quaternion_ref.Data(:,1);
q1_ref = out.quaternion_ref.Data(:,2);
q2_ref = out.quaternion_ref.Data(:,3);
q3_ref = out.quaternion_ref.Data(:,4);
q0 = out.quaternion.Data(:,1);
q1 = out.quaternion.Data(:,2);
q2 = out.quaternion.Data(:,3);
q3 = out.quaternion.Data(:,4);

subplot(3,1,1);
plot(time, q0_ref);
hold on; grid on;
plot(time, q0);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$q_0$', 'Interpreter', 'latex');
title('\textbf{Quaternion tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(4,1,2);
plot(time, q1_ref);
hold on; grid on;
plot(time, q1);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$q_1$', 'Interpreter', 'latex');
title('\textbf{Quaternion tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(4,1,3);
plot(time, q2_ref);
hold on; grid on;
plot(time, q2);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$q_2$', 'Interpreter', 'latex');
title('\textbf{Quaternion tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

subplot(4,1,4);
plot(time, q3_ref);
hold on; grid on;
plot(time, q3);
xlabel('t (s)', 'Interpreter', 'latex');
ylabel('$q_3$', 'Interpreter', 'latex');
title('\textbf{Quaternion tracking}', 'Interpreter', 'latex');
legend({'Reference', 'Output'}, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

%% Propellers speeds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w1 = out.propellers_speed.Data(:,1);
w2 = out.propellers_speed.Data(:,2);
w3 = out.propellers_speed.Data(:,3);
w4 = out.propellers_speed.Data(:,4);

rad_sec_to_rpm = 30/pi;

figure(4); clf;
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
pos_ref = out.position_ned_ref.Data;
pos = out.position_ned.Data;

figure(5);
clf;

dtplot = 0.1;
N = length(time);
dk = floor(dtplot / simulation_step);
i_span = 1:dk:N;
Ni = length(i_span);

trajectory_enu_ref = zeros(3, Ni);
trajectory_enu = zeros(3, Ni);
for i = 1:N
    % Reference (converted in ENU - more intuitive)
    R_ned_to_enu = [1 0 0;0 -1 0; 0 0 -1];
    R_enu = R_ned_to_enu * R(:,:,i);
    R_enu_ref = R_ned_to_enu * R_ref(:,:,i);
    pos_enu = R_ned_to_enu * pos(i,:)';
    pos_enu_ref = R_ned_to_enu * pos_ref(i,:)';
    trajectory_enu_ref(:,i) = pos_enu_ref;
    trajectory_enu(:,i) = pos_enu;
end

quadScale = 0.2;
displayAxes = 1;
axesScale = 1;
tem_traj_duration = 1;
tem_traj_it = floor(tem_traj_duration / simulation_step);
for i = i_span
    % Reference (converted in ENU - more intuitive)
    R_ned_to_enu = [1 0 0;0 -1 0; 0 0 -1];
    R_enu = R_ned_to_enu * R(:,:,i);
    R_enu_ref = R_ned_to_enu * R_ref(:,:,i);
    pos_enu = trajectory_enu(:,i);
    pos_enu_ref = trajectory_enu_ref(:,i);
    
    figure(5);
    clf;
    
    % Trajectory
    subplot(1,2,1);
    hold on;
    plot3(trajectory_enu_ref(1,1:i), trajectory_enu_ref(2,1:i), trajectory_enu_ref(3,1:i), ...
          'r', 'linewidth', 1.5);
    plot3(pos_enu_ref(1), pos_enu_ref(2), pos_enu_ref(3), ...
          'r+', 'linewidth', 2);
    plot3(trajectory_enu(1,1:i), trajectory_enu(2,1:i), trajectory_enu(3,1:i), ...
          'b', 'linewidth', 1.5);
    plot3(pos_enu(1), pos_enu(2), pos_enu(3), ...
          'bo', 'linewidth', 2);
    
    xlabel('x', 'Interpreter', 'latex');
    ylabel('y', 'Interpreter', 'latex');
    zlabel('z', 'Interpreter', 'latex');
    xlim([-2 2]*2);
    ylim([-2 2]*2);
    zlim([-2 2]*2);
    daspect([1 1 1]);
    set(gca, 'view', [117 28]);
    grid on;
    title_fig = ['\textbf{Quadrotor trajectory} $t=', num2str(time(i)), '\mathrm{s}$'];
    title(title_fig, 'Interpreter', 'latex');
    
    % Pose
    subplot(1,2,2);   
    
    draw.axes(R_enu_ref, pos_enu, 'm', axesScale);
    draw.axes(R_enu, pos_enu, 'c', axesScale);
    
    % Drone    
    quadrotor.draw(R_enu, pos_enu, quadScale, 'b');
    
    % trajectory
    i_start = max([1 i-tem_traj_it]);
    plot3(trajectory_enu(1,i_start:i), trajectory_enu(2,i_start:i), trajectory_enu(3,i_start:i), ...
          'b', 'linewidth', 1.5);
    plot3(trajectory_enu_ref(1,i_start:i), trajectory_enu_ref(2,i_start:i), trajectory_enu_ref(3,i_start:i), ...
          'r', 'linewidth', 1.5);
    plot3(trajectory_enu_ref(1,i), trajectory_enu_ref(2,i), trajectory_enu_ref(3,i), ...
          'r+', 'linewidth', 2);
    xlabel('x', 'Interpreter', 'latex');
    ylabel('y', 'Interpreter', 'latex');
    zlabel('z', 'Interpreter', 'latex');
    daspect([1 1 1]);
    set(gca, 'view', [117 28]);
    grid on;
    xlim([-1 1]*axesScale + pos_enu(1));
    ylim([-1 1]*axesScale + pos_enu(2));
    zlim([-1 1]*axesScale + pos_enu(3));
    
    title_fig = ['\textbf{Quadrotor pose at time} $t=', num2str(round(time(i))), '\mathrm{s}$'];
    title(title_fig, 'Interpreter', 'latex');
    drawnow;
end