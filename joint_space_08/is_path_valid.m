function [is_valid, collision_index] = is_path_valid(q_path, joint_path, voxel_grid, range)
% Funkce ověří, zda je celá q_path bez kolizí
% q_path: Nx6 matice kloubových konfigurací

is_valid = true;
collision_index = 0;
num_points = size(q_path,2);

for i = 1:num_points
    joint_positions = squeeze(joint_path(i, 1:4, :));  % [6x3]
    
    % Kontrola kolize kloubů s překážkami
        if is_joint_in_collision(joint_positions, voxel_grid, range)
            is_valid = false;
            collision_index = i;
            return;
        end
        % Kontrola kolize pomocí modelu robota
        if is_robot_model_in_collision(joint_positions, voxel_grid, range)
            is_valid = false;
            collision_index = i;
            return;
        end
end
disp('bez kolize')

end

