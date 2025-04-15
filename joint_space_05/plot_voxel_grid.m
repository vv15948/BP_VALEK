function plot_voxel_grid(voxel_grid,range, voxel_size,joint_angles,robot, path,traveled_path)

%Vizualizace:
% voxel gridu, robota UR5e v aktuální konfigurace
% Projeté a naplánované cesty

t = tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
x_range = range(1,:);
y_range = range(2,:);
z_range = range(3,:);


[x_vox, y_vox, z_vox] = ind2sub(size(voxel_grid), find(voxel_grid == 1));
[x_start, y_start, z_start] = ind2sub(size(voxel_grid), find(voxel_grid == 2)); % Start
[x_goal, y_goal, z_goal] = ind2sub(size(voxel_grid), find(voxel_grid == 3)); % Cíl

voxel_size = voxel_size*100;

nexttile
    cla;
    hold on;
         show(robot, joint_angles, 'PreservePlot', true, 'Frames', 'off', 'Parent', gca);
    scatter3(x_range(x_vox), y_range(y_vox), z_range(z_vox), voxel_size, 'r', 'filled');
    scatter3(x_range(x_start), y_range(y_start), z_range(z_start), 200, 'b', 'filled'); % Start - modrá
    scatter3(x_range(x_goal), y_range(y_goal), z_range(z_goal), 200, 'g', 'filled'); % Cíl - zelená
    if ~isempty(path)
        plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2); % Tyrkysová trajektorie
    end
    if ~isempty(traveled_path)
        plot3(traveled_path(:,1), traveled_path(:,2), traveled_path(:,3), 'y-', 'LineWidth', 2); % Tyrkysová trajektorie
    end
    

    title('3D 1');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(45, 30);
    xlim([-0.85 0.85]); ylim([1.2 2.9]); zlim([-0.85 0.85]);
    axis manual;
    axis equal; grid on;



    nexttile
    cla;
        hold on;
        show(robot, joint_angles, 'PreservePlot', true, 'Frames', 'off', 'Parent', gca);
    scatter3(x_range(x_vox), y_range(y_vox), z_range(z_vox), voxel_size, 'r', 'filled');
    scatter3(x_range(x_start), y_range(y_start), z_range(z_start), 200, 'b', 'filled');
    scatter3(x_range(x_goal), y_range(y_goal), z_range(z_goal), 200, 'g', 'filled');
    if ~isempty(path)
        plot3(path(:,1), path(:,2), path(:,3), 'c-', 'LineWidth', 2); % Tyrkysová trajektorie
    end
    if ~isempty(traveled_path)
        plot3(traveled_path(:,1), traveled_path(:,2), traveled_path(:,3), 'y-', 'LineWidth', 2); % Tyrkysová trajektorie
    end


     title(sprintf('3D 2'));
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    view(135, 30);
    xlim([-0.85 0.85]); ylim([1.2 2.9]); zlim([-0.85 0.85]);
    axis manual;
    axis equal; grid on;

end

