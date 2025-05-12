function voxel_grid = add_cylinder_to_voxel_grid(voxel_grid, x_range, y_range, z_range, start_point, end_point, radius)
    
    voxel_size = x_range(2) - x_range(1);
    x_min = x_range(1);
    y_min = y_range(1);
    z_min = z_range(1);

    Nx = length(x_range);
    Ny = length(y_range);
    Nz = length(z_range);

    % Směr a délka válce
    v = end_point - start_point;
    v_length = norm(v);
    if v_length == 0
        return;
    end
    % jednotkový vektor směru
    v_unit = v / v_length;

    % Výpočet kvádru kolem válce s rezervou 2 radiusů
    all_points = [start_point; end_point];
    min_bound = min(all_points) - radius;
    max_bound = max(all_points) + radius;

    % Převod souřadnic na indexy mřížky
    to_idx = @(val, axis_min) round((val - axis_min) / voxel_size) + 1;

    ix1 = max(1, to_idx(min_bound(1), x_min));
    ix2 = min(Nx, to_idx(max_bound(1), x_min));
    iy1 = max(1, to_idx(min_bound(2), y_min));
    iy2 = min(Ny, to_idx(max_bound(2), y_min));
    iz1 = max(1, to_idx(min_bound(3), z_min));
    iz2 = min(Nz, to_idx(max_bound(3), z_min));

    % Průchod pouze kolem oblasti válce
    % Neprochází se celý voxel_grid - urychlení výpočtu
    for i = ix1:ix2
        for j = iy1:iy2
            for k = iz1:iz2
                point = [x_range(i), y_range(j), z_range(k)];
                vec = point - start_point;

                % Projekce bodu na osu válce pomocí skalárního součinu
                projection_length = dot(vec, v_unit);

                % Kontrola zda bod leží na délce válce
                if projection_length < 0 || projection_length > v_length
                    continue;
                end

                % Vzdálenost bodu od osy válce
                nearest_point = start_point + projection_length * v_unit;
                distance = norm(point - nearest_point);

                if distance <= radius
                    voxel_grid(i, j, k) = 1;
                end
            end
        end
    end
end
