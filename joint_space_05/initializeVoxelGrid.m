function [range, voxel_grid] = initializeVoxelGrid(voxel_size)
% Definice voxelové mřížky

x_range = -0.85:voxel_size:0.85;  
y_range = -0.85:voxel_size:0.85;    
z_range = -0.15:voxel_size:1.55;

voxel_grid = zeros(length(x_range), length(y_range), length(z_range)); % Inicializace 3D mřížky
range = [x_range;y_range;z_range];
end