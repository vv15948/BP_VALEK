clc;
clear;
close all;

% ====== NASTAVENÍ ======
folder = 'snapshot_data';
output_folder = 'voxel_framesss';
resolution = 200;  % DPI pro export
robot = loadrobot('universalUR5e', 'DataFormat', 'row'); % Robot definice

if exist(output_folder, 'dir')
    try
        delete(fullfile(output_folder, '*'));
        pause(0.1);
        rmdir(output_folder,  's');
        pause(0.1);
    catch ME
        warning("smazani selhalo")
    end
end
mkdir(output_folder);

% ====== ZPRACOVÁNÍ VŠECH SNAPSHOTŮ ======
files = dir(fullfile(folder, 'snapshot_*.mat'));
[~, idx] = sort(arrayfun(@(f) sscanf(f.name, 'snapshot_%d.mat'), files));
files = files(idx);

for i = 1:length(files)
    data = load(fullfile(folder, files(i).name));
    
    % === Vstupní proměnné ===
    voxel_grid = data.voxel_grid;
    range = data.range;
    voxel_size = data.voxel_size;
    timestamp = data.timestamp;
    joint_angles = data.q_now;  % první konfigurace
    path = squeeze(data.actual_joint_position_path_spline(:,7,:));  % TCP trajektorie
    traveled_path = data.traveled_path;

    fig = figure('Visible', 'off');
    fig.Position(3:4) = [1707, 960];  % Full HD

    plot_voxel_grid(voxel_grid, range, voxel_size, joint_angles, robot, path, traveled_path);

    filename = fullfile(output_folder, sprintf('voxel_%.3f.png', timestamp));
    exportgraphics(fig, filename, 'Resolution', resolution);
    close(fig);
    fprintf("✅ Uloženo: %s\n", filename);
end
