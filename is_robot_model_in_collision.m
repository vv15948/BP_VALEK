function collision = is_robot_model_in_collision(positions, voxel_grid, range,dir)
% Rychlejší kontrola kolizí pomocí válců

    % Předdefinované parametry pro jednotlivé úseky
    radii = [0.08, 0.07, 0.06, 0.06, 0.06];     % Poloměry válců
    offsets = [0.072, 0.06, 0.03, 0, 0];        % Délka prodloužení válce v obou směrech

    % Úprava počátečního směru pro rameno 1 (speciální offset)
    dir1 = [dir(2), -dir(1), dir(3)];
    offset_dir1 = dir1 .* 0.14;

    for i = 1:5
        radius = radii(i);
        offset = offsets(i);

        % Výpočet start_point a end_point pro válce
        if i == 1
            start_point = positions(i,:) + offset_dir1;
            end_point = positions(i+1,:) + offset_dir1;
        else
            start_point = positions(i,:);
            end_point = positions(i+1,:);
        end

        % Výpočet směrového vektoru válce
        direction = end_point - start_point;
        norm_dir = direction / norm(direction);

        % Ve směru válce doplnění prodloužení
        offset_vec = norm_dir * offset;
        start_point = start_point - offset_vec;
        end_point = end_point + offset_vec;

        % Vytvoření masky s konkrétním válcem
        cylinder_mask = add_cylinder_to_voxel_grid(false(size(voxel_grid)), ...
            range(1,:), range(2,:), range(3,:), ...
            start_point, end_point, radius);
        
        % Porovnání s voxel_grid, zda válec neleží v překážce
        if any(cylinder_mask(:) & voxel_grid(:))
            collision = true;
            return;
        end
    end
    % Pokud vše projde, tak nebyla detekována žádná kolize
    collision = false;
end
