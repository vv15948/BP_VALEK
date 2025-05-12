function [is_valid, collision_index] = is_path_valid(q_path, joint_path, voxel_grid, range,dir)
% Funkce pro ověření zda je naplánovaná cesta q_path bez kolizí

    % Inicializace
    is_valid = true;
    collision_index = 0;
    num_points = size(q_path,2);
    
    % Pokud je cesta krátká rovnou ji vracím a kontroluji zda již není dostatečně blízko cíli
    if num_points <= 2
        is_valid = false;
        return;
    end
    
    % Procházení postupně všech konfigurací cesty
    for i = 2:num_points-1
        joint_positions = squeeze(joint_path(i, :, :));  % [6x3]
        
        % Kontrola kolize kloubů s překážkami
            if is_joint_in_collision(joint_positions, voxel_grid, range)
                is_valid = false;
                collision_index = i;
                return;
            end

            % Kontrola kolize pomocí modelu robota
            if is_robot_model_in_collision(joint_positions, voxel_grid, range,dir(i,:))
                is_valid = false;
                collision_index = i;
                return;
            end
    end

    % Pokud vše prošlo, cesta je validní a vracíme se zpět

end

