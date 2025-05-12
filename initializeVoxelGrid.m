function [range, voxel_grid] = initializeVoxelGrid(voxel_size)

% Definice voxelové mřížky

% Rozsahy
x_range = -0.85:voxel_size:0.85;  
y_range = -0.85:voxel_size:0.85;
z_range = -0.15:voxel_size:1.55;

% Inicializace 3D mřížky
voxel_grid = zeros(length(x_range), length(y_range), length(z_range)); 
range = [x_range;y_range;z_range];

% Doplnění překážek do vrcholů matice (pro plot)
voxel_grid(1,1,1) = 1;
voxel_grid(1,1,end) = 1;
voxel_grid(1,end,1) = 1;
voxel_grid(1,end,end) = 1;
voxel_grid(end,1,1) = 1;
voxel_grid(end,1,end) = 1;
voxel_grid(end,end,1) = 1;
voxel_grid(end,end,end) = 1;
end