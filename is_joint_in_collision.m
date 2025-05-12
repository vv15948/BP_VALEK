function collision = is_joint_in_collision(positions, voxel_grid, range)
% Vyhodnocení kolize kloubů s překážkami ve voxel gridu

    voxel_size = range(1,2) - range(1,1);

    % Převod pozic kloubů do indexů v mřížce
    ix = round((positions(:,1) - range(1,1)) / voxel_size) + 1;
    iy = round((positions(:,2) - range(2,1)) / voxel_size) + 1;
    iz = round((positions(:,3) - range(3,1)) / voxel_size) + 1;

    % Vybrat jen indexy, které jsou platné a leží uvnitř voxel_grid
    valid = ix >= 1 & ix <= size(voxel_grid,1) & iy >= 1 & iy <= size(voxel_grid,2) & iz >= 1 & iz <= size(voxel_grid,3);

    % Lineární indexy jen pro platné body
    linear_idx = sub2ind(size(voxel_grid), ix(valid), iy(valid), iz(valid));

    % Vyhodnocení kolize
    collision = any(voxel_grid(linear_idx) == 1);
end