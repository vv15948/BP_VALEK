function voxel_grid = fill_sphere(voxel_grid, x_range, y_range, z_range, center, radius)
    
    voxel_size = x_range(2) - x_range(1);  
    x_min = x_range(1);
    y_min = y_range(1);
    z_min = z_range(1);

    Nx = length(x_range);
    Ny = length(y_range);
    Nz = length(z_range);

    % Převod souřadnic na indexy mřížky
    to_idx = @(val, axis_min) round((val - axis_min) / voxel_size) + 1;

    % Indexový rozsah kolem sféry
    ix1 = max(1, to_idx(center(1) - radius, x_min));
    ix2 = min(Nx, to_idx(center(1) + radius, x_min));
    iy1 = max(1, to_idx(center(2) - radius, y_min));
    iy2 = min(Ny, to_idx(center(2) + radius, y_min));
    iz1 = max(1, to_idx(center(3) - radius, z_min));
    iz2 = min(Nz, to_idx(center(3) + radius, z_min));

    % Průchod pouze kolem oblasti koule
    % Neprochází se celý voxel_grid - urychlení výpočtu
    for i = ix1:ix2
        for j = iy1:iy2
            for k = iz1:iz2
                x = x_range(i);
                y = y_range(j);
                z = z_range(k);

                % Kontrola, zda je bod uvnitř koule
                if (x - center(1))^2 + (y - center(2))^2 + (z - center(3))^2 <= radius^2
                    voxel_grid(i, j, k) = 1;
                end
            end
        end
    end
end
