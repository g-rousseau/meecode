%% ZYX intrinsic Euler angles to quaternion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function quaternion = euler_angles_to_quaternion(euler_angles)

q_roll = [cos(0.5 * euler_angles.roll); sin(0.5 * euler_angles.roll); 0; 0];
q_pitch = [cos(0.5 * euler_angles.pitch); 0; sin(0.5 * euler_angles.pitch); 0];
q_yaw = [cos(0.5 * euler_angles.yaw); 0; 0; sin(0.5 * euler_angles.yaw)];

quaternion = utils.quaternion_to_matrix(q_yaw) * utils.quaternion_to_matrix(q_pitch) * q_roll;
end
