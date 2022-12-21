%% Quaternion to intrinsic ZYX Euler angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler_angles = rotation_matrix_to_euler_angles(R)

gimbalLock = false;
sin_pitch = -R(3,1);
if 1 - abs(sin_pitch) < eps % Near gimbal lock
    gimbalLock = true;
end

if gimbalLock % On gimbal lock, bank angle (roll) is set to 0
    euler_angles.roll = 0;
    euler_angles.yaw = atan2(R(2,3), R(1,3));
else
    euler_angles.roll = atan2(R(3,2), R(3,3));
    euler_angles.yaw = atan2(R(2,1), R(1,1));
end
euler_angles.pitch = asin(sin_pitch);
end
