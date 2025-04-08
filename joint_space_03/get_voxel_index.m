function idx = get_voxel_index(point, range, voxel_grid)
    voxel_size = range(1,2) - range(1,1);
    ix = round((point(1) - range(1,1)) / voxel_size) + 1;
    iy = round((point(2) - range(2,1)) / voxel_size) + 1;
    iz = round((point(3) - range(3,1)) / voxel_size) + 1;

    if ix < 1 || iy < 1 || iz < 1 || ...
       ix > size(voxel_grid,1) || iy > size(voxel_grid,2) || iz > size(voxel_grid,3)
        idx = [];
    else
        idx = [ix, iy, iz];
    end
end