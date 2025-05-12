function voxel_grid = add_start_and_goal(voxel_grid, range, start_point, goal_point)

x_range = range(1,:);
y_range = range(2,:);
z_range = range(3,:);

% Nejbližší prvek matice ke startu
% ve voxel_grid = 2
[~, x_start] = min(abs(x_range - start_point(1)));
[~, y_start] = min(abs(y_range - start_point(2)));
[~, z_start] = min(abs(z_range - start_point(3)));
voxel_grid(x_start, y_start, z_start) = 2;

% Nejbližší prvek matice k cíli
% ve voxel_grid = 3
[~, x_goal] = min(abs(x_range - goal_point(1)));
[~, y_goal] = min(abs(y_range - goal_point(2)));
[~, z_goal] = min(abs(z_range - goal_point(3)));
voxel_grid(x_goal, y_goal, z_goal) = 3;  

end
