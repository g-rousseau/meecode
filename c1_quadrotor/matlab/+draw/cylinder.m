%% Draw function: 3D cylinder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gauthier ROUSSEAU
% Parrot Drones - CentraleSupelec
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cylinder_handle = ...
             cylinder(lower_disk_center,... % Position of the center of the lower disk
                      upper_disk_center,... % Position of the center of the upper disk
                      radius_x,...          % semi-axis - x
                      radius_y,...          % semi axis - y
                      faces_cnt,...         % Number of faces of the cylinder
                      color,...             % Color of the object
                      is_closed,...         % False : Open cylinder  -  True : is_closed cylinder
                      mesh_alpha,...        % Alpha of the mesh
                      edges_alpha,...       % Alpha of the edges
                      faces_alpha)          % Alpha of the faces
%
%

% Calculating the length of the cylinder
length_cyl = norm(upper_disk_center - lower_disk_center);

% Creating a circle in the YZ plane
t = linspace(0, 2*pi, faces_cnt)';
x2 = radius_x * cos(t);
x3 = radius_y * sin(t);

% Creating the points in the X-Direction
x1 = [0, length_cyl];

% Creating (Extruding) the cylinder points in the X-Directions
xx1 = repmat(x1,length(x2),1);
xx2 = repmat(x2,1,2);
xx3 = repmat(x3,1,2);

% Drawing two filled cirlces to close the cylinder
if is_closed
    hold on
    bottom_handle = fill3(xx1(:,1), xx2(:,1), xx3(:,1), 'r');
    top_handle = fill3(xx1(:,2), xx2(:,2), xx3(:,2), 'r');
end

% Plotting the cylinder along the X-Direction with required length starting
% from Origin
side_handle = mesh(xx1, xx2, xx3);

% Angle between the x direction and the required direction
direction = upper_disk_center - lower_disk_center;
direction = direction / norm(direction);
rot_angle = rad2deg(acos(direction(1)));

% Finding the axis of rotation (single rotation) to rotate the cylinder in
% x-direction to the required arbitrary direction through cross product
rot_axis = cross([1 0 0], (upper_disk_center-lower_disk_center));

% Rotating the plotted cylinder and the end plate circles to the required
% angles
if rot_angle~=0 % Rotation is not needed if required direction is along X
    rotate(side_handle, rot_axis, rot_angle,[0 0 0])
    if is_closed
        rotate(bottom_handle, rot_axis, rot_angle, [0 0 0])
        rotate(top_handle ,rot_axis, rot_angle, [0 0 0])
    end
end

% Till now cylinder has only been aligned with the required direction, but
% position starts from the origin. so it will now be shifted to the right
% position
if is_closed
    set(bottom_handle, 'XData', get(bottom_handle,'XData') + lower_disk_center(1))
    set(bottom_handle, 'YData', get(bottom_handle,'YData') + lower_disk_center(2))
    set(bottom_handle, 'ZData', get(bottom_handle,'ZData') + lower_disk_center(3))
    set(top_handle, 'XData', get(top_handle,'XData') + lower_disk_center(1))
    set(top_handle, 'YData', get(top_handle,'YData') + lower_disk_center(2))
    set(top_handle, 'ZData', get(top_handle,'ZData') + lower_disk_center(3))
end
set(side_handle, 'XData', get(side_handle, 'XData') + lower_disk_center(1))
set(side_handle, 'YData', get(side_handle, 'YData') + lower_disk_center(2))
set(side_handle, 'ZData', get(side_handle, 'ZData') + lower_disk_center(3))

% Setting the color to the cylinder and the end plates
set(side_handle,'FaceColor', color)
if is_closed
    set([bottom_handle top_handle], 'FaceColor', color)
    set([bottom_handle top_handle], 'EdgeAlpha', edges_alpha)
    set([bottom_handle top_handle], 'FaceAlpha', faces_alpha)
else
    bottom_handle = [];
    top_handle = [];
end

set(side_handle, 'EdgeAlpha', mesh_alpha)
set(side_handle, 'FaceAlpha', faces_alpha)

cylinder_handle(1) = side_handle;
if is_closed
    cylinder_handle(2) = top_handle;
    cylinder_handle(3) = bottom_handle;
end
end