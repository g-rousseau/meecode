%% Quaternion to intrinsic ZYX Euler angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function euler_angles = quaternion_to_euler_angles(q)

gimbalLock = 0;
sin_pitch = 2 * (q(1)* q(3) - q(2)*q(4));
if abs(abs(sin_pitch)-1) <= 10*eps % Near gimbal lock
    gimbalLock = 1;
end

if gimbalLock % On gimbal lock, bank angle (roll) is set to 0
    euler_angles.roll = 0;
    euler_angles.yaw = atan2(-2 * (q(2)* q(3) - q(1)*q(4)), 1 - 2 * (q(2)^2 + q(4)^2));
else
    euler_angles.roll = atan2(2 * (q(1)*q(2) +  q(3)*q(4)), 1 - 2 * (q(2)^2 +  q(3)^2));
    euler_angles.yaw = atan2(2 * (q(1)*q(4) + q(2)* q(3)), 1 - 2 * ( q(3)^2 + q(4)^2));
end
euler_angles.pitch = asin(sin_pitch);
end
