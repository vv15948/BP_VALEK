function collision = is_joint_in_collision(positions, voxel_grid, range)
    collision = false;
    for i = 1:size(positions,1)
        idx = get_voxel_index(positions(i,:), range, voxel_grid);
        if isempty(idx) || voxel_grid(idx(1), idx(2), idx(3)) == 1
            collision = true;
            return;
        end
    end
end