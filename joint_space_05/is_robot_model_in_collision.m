function collision = is_robot_model_in_collision(positions, voxel_grid, range)
% Kontrola kolizí modelu robota (UR5e) pomocí válců mezi klouby
% Všechny válce se přidají do jednoho gridu a pak se porovná s voxel_grid

    % Inicializace prázdné mřížky 
    robot_grid = false(size(voxel_grid));
    
    % Poloměr válců
    radius = 0.015;

    % Pro každý úsek mezi klouby vytvoření válce
    for i = 1:(size(positions,1) - 1)
        start_point = positions(i,:);
        end_point = positions(i+1,:);
        
        % Přidání válce do gridu 
        cylinder_mask = add_cylinder_to_voxel_grid(false(size(voxel_grid)), ...
            range(1,:), range(2,:), range(3,:), ...
            start_point, end_point, radius);

        % Akumulace objemu robota
        robot_grid = robot_grid | cylinder_mask;
    end

    % Jedna finální kolizní kontrola
    collision = any(robot_grid(:) & voxel_grid(:));
end