function voxel_mask = generate_voxel_mask(range, box_params, cutout_params,voxel_size, voxel_grid)

    x_min = range(1,1);
    y_min = range(2,1);
    z_min = range(3,1);

    voxel_mask = voxel_grid;

    % Pomocná funkce pro převod souřadnic do indexu v mřížce
    to_idx = @(val, axis_min) round((val - axis_min) / voxel_size) + 1;

    % Přidání kvádrů jako překážek
    for i = 1:size(box_params, 1)
        ix1 = to_idx(box_params(i,1), x_min);
        ix2 = to_idx(box_params(i,2), x_min);
        iy1 = to_idx(box_params(i,3), y_min);
        iy2 = to_idx(box_params(i,4), y_min);
        iz1 = to_idx(box_params(i,5), z_min);
        iz2 = to_idx(box_params(i,6), z_min);

        voxel_mask(ix1:ix2, iy1:iy2, iz1:iz2) = 1;
    end

    % Odebrání výřezů
    for i = 1:size(cutout_params, 1)
        ix1 = to_idx(cutout_params(i,1), x_min);
        ix2 = to_idx(cutout_params(i,2), x_min);
        iy1 = to_idx(cutout_params(i,3), y_min);
        iy2 = to_idx(cutout_params(i,4), y_min);
        iz1 = to_idx(cutout_params(i,5), z_min);
        iz2 = to_idx(cutout_params(i,6), z_min);

        voxel_mask(ix1:ix2, iy1:iy2, iz1:iz2) = 0;
    end
end

