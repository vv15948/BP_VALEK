function collision = is_robot_model_in_collision(positions, voxel_grid, range)
% Rychlejší kontrola kolizí pomocí válců + sféry na posledním kloubu

    % Poloměr válců a koule
    radius = 0.05;
    sradius = 0.05;

    % Kontrola každého válce zvlášť
    for i = 1:(size(positions,1) - 1)
        start_point = positions(i,:);
        end_point = positions(i+1,:);

        % Vytvoříme masku jen pro tento válec
        cylinder_mask = add_cylinder_to_voxel_grid(false(size(voxel_grid)), ...
            range(1,:), range(2,:), range(3,:), ...
            start_point, end_point, radius);

        % Pokud došlo ke kolizi – ihned skončíme
        if any(cylinder_mask(:) & voxel_grid(:))
            collision = true;
            return;
        end
    end

    % Přidání koule na poslední kloub
    center = positions(end,:);
    sphere_mask = fill_sphere(false(size(voxel_grid)), ...
        range(1,:), range(2,:), range(3,:), center, sradius);

    if any(sphere_mask(:) & voxel_grid(:))
        collision = true;
        return;
    end

    % Pokud jsme prošli vše bez kolize:
    collision = false;
end
