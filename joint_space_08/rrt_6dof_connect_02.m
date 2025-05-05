function [q_path, joint_path, u1, u2, goal_flag,dirout] = rrt_6dof_connect_02(q_start, q_goal, voxel_grid, range)



% inicializace
u1 = 0; u2 = 0; goal_flag = false;
goal_idx_start = 0; goal_idx_goal = 0;


% Nastavení parametrů
max_step_per_joint = [
    0.25;   % q1
    0.2;    % q2
    0.15;   % q3
    0.1;
    0.1;
    0.1
]*0.8;
max_iter = 3000;
goal_tolerance = 0.2;
joint_limits = repmat([-2*pi, 2*pi], 6, 1);


% Startovní nastavení stromů
Q_start = q_start;
[fk_start(:,:,1), dir_start(1,:)] = forward_kinematics_model(q_start);
Q_goal = q_goal;
P_start = 0;
P_goal = 0;
[fk_goal(:,:,1), dir_goal(1,:)] = forward_kinematics_model(q_goal);
tree_direction = 1;
goal_bias = 0.4;


%% Hlavní smyčka 
for iter = 1:max_iter

    % Výběr náhodného bodu/cílového podle nastavení goal_bias
    if rand < goal_bias
        q_rand = (tree_direction == 1) * Q_goal(:,1) + (tree_direction == -1) * Q_start(:,1);
    else
        q_rand = joint_limits(:,1) + (joint_limits(:,2) - joint_limits(:,1)) .* rand(6,1);
    end

    % Rozšíření stromu ve směru ke q_rand
    [q_new_start, idx_nearest_s] = extend_tree_matrix(Q_start, q_rand, max_step_per_joint);
    if isempty(q_new_start), continue; end

    % Kolizní kontrola 
    [fk_pos_s,dir] = forward_kinematics_model(q_new_start);
    if any(fk_pos_s(:,3) < 0.1), continue; end
    if is_joint_in_collision(fk_pos_s, voxel_grid, range), u1 = u1 + 1; continue; end
    if is_robot_model_in_collision(fk_pos_s, voxel_grid, range,dir), u2 = u2 + 1; continue; end

    % Přidání nového bodu do stromu
    Q_start(:,end+1) = q_new_start;
    P_start(end+1) = idx_nearest_s;
    dir_start(end+1,:) = dir;
    fk_start(:,:,end+1) = fk_pos_s;


    % Vyhledání njebližšího bodu ve druhém stromě
    distances = vecnorm(Q_goal - q_new_start, 2, 1);
    [~, idx_nearest_g] = min(distances);
    q_near = Q_goal(:,idx_nearest_g);


    % Kolikrát se maximálně pokusíme udělat kroků od druhého stromu k
    % prvnímu
    max_s = 2;

    % Rozšiřování druhého stromu
    s=0;
    while true
        direction = q_new_start - q_near;
        dist = norm(direction);
        if dist < 1e-6, break; end

        % Vytvoření nového bodu
        q_new_goal = q_near + max_step_per_joint .* direction / dist;
% Ověření, zda se q_new_goal přibližuje ke q_goal (nevrací se zpět)
if norm(q_new_goal - q_goal) > norm(q_near - q_goal)+0.2
    break;  % pokračování by vedlo od cíle, ukončíme rozšiřování
end

        % Kolizní kontrola
        [fk_pos_g,dir] = forward_kinematics_model(q_new_goal);
        if any(fk_pos_g(:,3) < -0.15), break; end
        if is_joint_in_collision(fk_pos_g, voxel_grid, range), u1 = u1 + 1; break; end
        if is_robot_model_in_collision(fk_pos_g, voxel_grid, range,dir), u2 = u2 + 1; break; end

        % Přidání bodu do stromu
        Q_goal(:,end+1) = q_new_goal;
        P_goal(end+1) = idx_nearest_g;
        dir_goal(end+1,:) = dir;
        fk_goal(:,:,end+1) = fk_pos_g;

        idx_nearest_g = size(Q_goal,2);
        q_near = q_new_goal;

        % Nalezení nejbližšího bodu 1. stromu k právě vytvořenému
        distances_to_start = vecnorm(Q_start - q_new_goal, 2, 1);
        [min_dist, min_idx] = min(distances_to_start);

        % Kontrola, jestli už se můžou propojit
        if min_dist < goal_tolerance
            goal_idx_start = min_idx;
            goal_idx_goal = size(Q_goal,2);
            goal_flag = true;
            disp('Cíl dosažen');
            break;
        end


        s = s+1;
        if s > max_s
            break;
        end
    end

if goal_flag, break; end

    % Stromy se nepropojily, tak si vymění role
    [Q_start, Q_goal] = deal(Q_goal, Q_start);
    [P_start, P_goal] = deal(P_goal, P_start);
    [dir_start, dir_goal] = deal(dir_goal, dir_start);
    [fk_start, fk_goal] = deal(fk_goal, fk_start);
    tree_direction = -tree_direction;
end

% Pokud se stromy nepropojily
if goal_flag == false
    q_path = [q_start, q_start];
    joint_path = repmat(zeros(6,3), [2, 1, 1]);
    dirout = [dir_start(1,:); dir_start(1,:)];
    disp('Stojíme – cíl nenalezen.');
    return;
end

% Posloupnost, jak na sebe jednotlivé body ve stromech navazují
idx_s = reconstruct_path_matrix(P_start, goal_idx_start);
idx_g = reconstruct_path_matrix(P_goal, goal_idx_goal);

% Správné seřazení stromů start -> goal
if norm(Q_start(:,idx_s(1)) - q_start) > norm(Q_goal(:,idx_g(1)) - q_start)
    [Q_start, Q_goal] = deal(Q_goal, Q_start);
    [P_start, P_goal] = deal(P_goal, P_start);
    [dir_start, dir_goal] = deal(dir_goal, dir_start);
    [fk_start, fk_goal] = deal(fk_goal, fk_start);
    [idx_s, idx_g] = deal(idx_g, idx_s);
end

q_path = [Q_start(:,idx_s), fliplr(Q_goal(:,idx_g))];
dirout = [dir_start(idx_s,:); flipud(dir_goal(idx_g,:))];
fk_combined = cat(3, fk_start(:,:,idx_s), fk_goal(:,:,idx_g(end:-1:1)));
joint_path = permute(fk_combined, [3, 1, 2]);



end


%% rozšíření stromu Q k bodu q_target
% Najde nejbližší bod ve stromu a udělá krok tímto směrem

function [q_new, best_parent_idx] = extend_tree_matrix(Q, q_target, max_step_per_joint)

    % Nalezení nejbližšího bodu ke q_target
    distances = vecnorm(Q - q_target, 2, 1);
    [~, best_parent_idx] = min(distances);

    % Vytvoříme nový bod směrem k cíli z nejlepšího rodiče
    q_near = Q(:,best_parent_idx);
    dq = q_target-q_near;
    dq_clipped = max(min(dq, max_step_per_joint), -max_step_per_joint);
    q_new = q_near + dq_clipped;

    % Ochrana proti NaN
    if any(isnan(q_new))
        q_new = [];
    end
end



%% Rekonstrukce posloupnosti indexů od kořene ke koncovému bodu

function idx_path = reconstruct_path_matrix(parent_list, idx)
    idx_path = idx;

    % Zpětné procházení rodičů, až k 0
    while parent_list(idx) ~= 0
        idx = parent_list(idx);
        idx_path = [idx, idx_path]; % Seřazení od startu k cíli
    end
end



