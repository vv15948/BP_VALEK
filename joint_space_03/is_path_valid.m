function valid = is_path_valid(q_path, voxel_grid, range, q_start)
% Funkce ověří, zda je celá q_path bez kolizí
% q_path: Nx6 matice kloubových konfigurací

% Kontrola prázdného vstupu nebo že cesta = q_start
if isempty(q_path) || (size(q_path,1) == 1 && all(abs(q_path(1,:) - q_start) < 1e-6))
    valid = false;
    return;
end

valid = true;

for i = 1:size(q_path, 1)
    q = q_path(i, :);
    positions = forward_kinematics_ur5e(q); % 7x3 pozice všech kloubů + TCP

    if is_joint_in_collision(positions, voxel_grid, range)
        valid = false;
        return;
    end
end
end