function [q_path, ee_path] = rrt_6dof_basic(q_start, q_goal, max_iter, step_size, voxel_grid, range)


% Parametry
joint_limits = [
    -2*pi, 2*pi;  % shoulder_pan_joint
    -2*pi, 2*pi;  % shoulder_lift_joint
    -pi,    pi;   % elbow_joint
    -2*pi, 2*pi;  % wrist_1_joint
    -2*pi, 2*pi;  % wrist_2_joint
    -2*pi, 2*pi   % wrist_3_joint
];
num_joints = 6;
goal_tolerance = step_size;  % tolerance v joint space
bias_prob = 0.15;
success = false;

% Inicializace stromu
node(1).q = q_start;
node(1).parent = 0;

% Předalokace
q_path = [];
ee_path = [];

% Hlavní smyčka
for iter = 1:max_iter
   % 1. Náhodná konfigurace nebo bias na cíl
    if rand < bias_prob
        q_rand = q_goal;
    else
        q_rand = arrayfun(@(i) ...
            rand * (joint_limits(i,2) - joint_limits(i,1)) + joint_limits(i,1), ...
            1:num_joints);
    end

    % 2. Najdi nejbližší bod
    dists = arrayfun(@(n) joint_distance(n.q, q_rand), node);
    [~, idx_nearest] = min(dists);
    q_nearest = node(idx_nearest).q;

    % 3. Udělej krok směrem k q_rand
    direction = q_rand - q_nearest;
    direction = wrapToPi(direction); % vyřeší wraparound
    q_new = q_nearest + step_size * direction / norm(direction);

    % 4. Kontrola kolizí
    joint_positions = forward_kinematics_ur5e(q_new);

    if is_joint_in_collision(joint_positions, voxel_grid, range)
        continue;
    end

    % if is_robot_model_in_collision(joint_positions, voxel_grid, range)
    %     continue;
    % end

    % 5. Přidej do stromu
    new_idx = length(node) + 1;
    node(new_idx).q = q_new;
    node(new_idx).parent = idx_nearest;

    % 6. Cíl dosažen?
    if joint_distance(q_new, q_goal) < goal_tolerance
        node(end+1).q = q_goal;
        node(end).parent = new_idx;
        success = true;
        break;
    end
end

if ~success
    warning('RRT nedosáhl cílové konfigurace. Vracíme prázdnou cestu.');
    q_path = [];
    ee_path = [];
    return;
end



% 7. Rekonstrukce cesty
q_path = [];
current = length(node);
while current ~= 0
    q_path = [node(current).q; q_path];
    current = node(current).parent;
end

% 8. Výpočet trajektorie koncového bodu
ee_path = zeros(size(q_path,1), 3);
for i = 1:size(q_path,1)
    positions = forward_kinematics_ur5e(q_path(i,:));
    ee_path(i,:) = positions(end,:); % TCP = poslední kloub
end
end

%% Pomocné funkce
function d = joint_distance(q1, q2)
    diff = q2 - q1;
    diff = mod(diff + pi, 2*pi) - pi;  % wrap do (-pi, pi]
    d = norm(diff);
end




function collision = is_robot_model_in_collision(positions, voxel_grid, range)
    collision = false;
    for i = 1:(size(positions,1)-1)
        start_point = positions(i,:);
        end_point = positions(i+1,:);
        temp_grid = add_cylinder_to_voxel_grid(zeros(size(voxel_grid)), range(1,:), range(2,:), range(3,:), start_point, end_point, 0.01);
        if any(temp_grid(:) & voxel_grid(:))
            collision = true;
            return;
        end
    end
end



