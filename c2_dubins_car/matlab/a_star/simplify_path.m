%% Path simplication
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Optionnal course: Multi agents dynamic systems
% Lesson 5: Motion Planning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function simplified_path = simplify_path(path)
%
%

% store nodes where a direction change ocurrs
node_count = length(path.id);
simplified_index = zeros(1, node_count);
simplified_index(1) = 1;
simplified_node_cnt = 1;
for i = 2:(node_count-1)
    prev_coord = [path.i(i-1); path.j(i-1)];
    current_coord = [path.i(i); path.j(i)];
    next_coord = [path.i(i+1); path.j(i+1)];
    prev_dir = current_coord - prev_coord;
    next_dir = next_coord - current_coord;
    if any(next_dir ~= prev_dir)
        simplified_node_cnt = simplified_node_cnt + 1;
        simplified_index(simplified_node_cnt) = i;
    end
end
simplified_node_cnt = simplified_node_cnt + 1;
simplified_index(simplified_node_cnt) = node_count;
simplified_index = simplified_index(1:simplified_node_cnt);

% simplified path
simplified_path.id = path.id(simplified_index);
simplified_path.i = path.i(simplified_index);
simplified_path.j = path.j(simplified_index);
end