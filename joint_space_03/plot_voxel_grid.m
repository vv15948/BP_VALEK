% function plot_voxel_grid(voxel_grid,range, voxel_size, fps_compute)
% 
% x_range = range(1,:);
% y_range = range(2,:);
% z_range = range(3,:);
% 
% 
% [x_vox, y_vox, z_vox] = ind2sub(size(voxel_grid), find(voxel_grid == 1));
% [x_start, y_start, z_start] = ind2sub(size(voxel_grid), find(voxel_grid == 2)); % Start
% [x_goal, y_goal, z_goal] = ind2sub(size(voxel_grid), find(voxel_grid == 3)); % Cíl
% 
% voxel_size = voxel_size*100;
% subplot(1, 2, 1);
%     cla;
%     scatter3(x_range(x_vox), y_range(y_vox), z_range(z_vox), voxel_size, 'r', 'filled');
%     scatter3(x_range(x_start), y_range(y_start), z_range(z_start), 200, 'b', 'filled'); % Start - modrá
%     scatter3(x_range(x_goal), y_range(y_goal), z_range(z_goal), 200, 'g', 'filled'); % Cíl - zelená
%     hold on;
% 
%     title('3D 1');
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%     view(45, 30);
%     xlim([-0.85 0.85]); ylim([1.2 2.9]); zlim([-0.85 0.85]);
%     axis equal; grid on;
% 
% % Pohled z boku (X-Y-Z)
%     subplot(1, 2, 2);
%     cla;
%     scatter3(x_range(x_vox), y_range(y_vox), z_range(z_vox), voxel_size, 'r', 'filled');
%     scatter3(x_range(x_start), y_range(y_start), z_range(z_start), 200, 'b', 'filled');
%     scatter3(x_range(x_goal), y_range(y_goal), z_range(z_goal), 200, 'g', 'filled');
%     hold on;
% 
%      title(sprintf('3D pohled (135°, 30°) | fps: %.1f', fps_compute));
%     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
%     view(135, 30);
%     xlim([-0.85 0.85]); ylim([1.2 2.9]); zlim([-0.85 0.85]);
%     axis equal; grid on;
% 
% end

function plot_voxel_grid(voxel_grid,range, voxel_size, fps_compute,joint_angles,robot, path)

x_range = range(1,:);
y_range = range(2,:);
z_range = range(3,:);


[x_vox, y_vox, z_vox] = ind2sub(size(voxel_grid), find(voxel_grid == 1));
[x_start, y_start, z_start] = ind2sub(size(voxel_grid), find(voxel_grid == 2)); % Start
[x_goal, y_goal, z_goal] = ind2sub(size(voxel_grid), find(voxel_grid == 3)); % Cíl

voxel_size = voxel_size*100;
subplot(1, 2, 1);
    cla;
    scatter3(x_range(x_vox), y_range(y_vox), z_range(z_vox), voxel_size, 'r', 'filled');
    scatter3(x_range(x_start), y_range(y_start), z_range(z_start), 200, 'b', 'filled'); % Start - modrá
    scatter3(x_range(x_goal), y_range(y_goal), z_range(z_goal), 200, 'g', 'filled'); % Cíl - zelená
    if ~isempty(path)
        plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2); % Tyrkysová trajektorie
    end
    hold on;

    title('3D 1');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(45, 30);
    xlim([-0.85 0.85]); ylim([1.2 2.9]); zlim([-0.85 0.85]);
    axis equal; grid on;
 % Vykresli robota přímo do subplotu 1
    show(robot, joint_angles, 'PreservePlot', true, 'Frames', 'off', 'Parent', gca);

    % Pohled z boku (X-Y-Z)
    subplot(1, 2, 2);
    cla;
    scatter3(x_range(x_vox), y_range(y_vox), z_range(z_vox), voxel_size, 'r', 'filled');
    scatter3(x_range(x_start), y_range(y_start), z_range(z_start), 200, 'b', 'filled');
    scatter3(x_range(x_goal), y_range(y_goal), z_range(z_goal), 200, 'g', 'filled');
    if ~isempty(path)
        plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2); % Tyrkysová trajektorie
    end
    hold on;

     title(sprintf('3D pohled (135°, 30°) | fps: %.1f', fps_compute));
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(135, 30);
    xlim([-0.85 0.85]); ylim([1.2 2.9]); zlim([-0.85 0.85]);
    axis equal; grid on;
% Vykresli robota přímo do subplotu 2
    show(robot, joint_angles, 'PreservePlot', true, 'Frames', 'off', 'Parent', gca);
end

