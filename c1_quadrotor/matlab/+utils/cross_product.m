%% Cross product
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uxv = cross_product(u, v)
    uxv = [u(2)*v(3) - u(3)*v(2)
           u(3)*v(1) - u(1)*v(3)
           u(1)*v(2) - u(2)*v(1)];
end