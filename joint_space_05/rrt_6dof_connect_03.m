function [q_path, joint_path, u1, u2, goal_flag] = rrt_6dof_connect_03(q_start, q_goal, voxel_grid, range)
% Vstupy:
%   q_start      - počáteční kloubová konfigurace (6x1)
%   q_goal       - cílová kloubová konfigurace (6x1)
%   voxel_grid   - voxelová mřížka s překážkami
%   range        - rozsahy jednotlivých os
%
% Výstupy:
%   q_path       - naplánovaná cesta v kloubovém prostoru (6xN)
%   joint_path   - poloha všech kloubů robota v naplánované cestě (Nx6x3)
%   u1           - počet bodů zamítnutých kvůli kolizi kloubů
%   u2           - počet bodů zamítnutých kvůli kolizi modelu robota
%   goal_flag    - true, pokud se podařilo propojit start a cíl


% inicializace
u1 = 0; % počet bodů, které neprošly 1. kolizní fází 
u2 = 0; % počet bodů, které neprošly 2. kolizní fází 
goal_flag = false;
goal_idx_start = 0;
goal_idx_goal = 0;

% Parametry
step_size = 0.3;
max_iter = 3000;
goal_tolerance = 0.3;
joint_limits = repmat([-2*pi, 2*pi], 6, 1);


% Inicializace stromů
Q_start = q_start;
Q_goal = q_goal;
P_start = 0;
P_goal = 0;
J_start(:,:,1) = forward_kinematics_ur5e(q_start);
J_goal(:,:,1)  = forward_kinematics_ur5e(q_goal);
tree_direction = 1;
goal_bias = 0.0;


%% hlavní smyčka
for iter = 1:max_iter

    %% generování náhodné konfigurace  
    if rand < goal_bias
        if tree_direction == 1
            q_rand = q_goal;
        else
            q_rand = q_start;
        end
    else
        q_rand = joint_limits(:,1) + (joint_limits(:,2) - joint_limits(:,1)) .* rand(6,1);
    end


    %% Rozšíření stromu START směrem ke q_rand
    [q_new_start, idx_nearest_s] = extend_tree_matrix(Q_start, q_rand, step_size);
    if isempty(q_new_start), continue; end

    % Kontrola zda je bod validní
    fk_new_s = forward_kinematics_ur5e(q_new_start);
    if any(fk_new_s(:,3) < -0.15)
        continue;
    end
    % Kontrola kloubů v prostoru
    if is_joint_in_collision(fk_new_s(2:7,:), voxel_grid, range)
       u1 = u1 + 1;
       continue;
    end
    % Kontrola válcového modelu robota
    if is_robot_model_in_collision(fk_new_s(2:7,:), voxel_grid, range)
       u2 = u2 + 1;
        continue;
    end

    % Přidání bodu do stromu
    Q_start(:,end+1) = q_new_start;
    P_start(end+1) = idx_nearest_s;
    J_start(:,:,end+1) = fk_new_s;

    %% Rozšíření stromu GOAL
    % Nalezení nejbližšího bodu GOAL stromu ke q_new_start
    distances = vecnorm(Q_goal - q_new_start, 2, 1);
    [~, idx_nearest_g] = min(distances);
    q_near = Q_goal(:,idx_nearest_g);

    % Kroky ve směru ke q_new_start
    while true
        direction = q_new_start - q_near;
        dist = norm(direction);
        if dist < 1e-6, break; end

        q_new_goal = q_near + step_size * direction / dist;

        fk_new_g = forward_kinematics_ur5e(q_new_goal);
        if any(fk_new_g(:,3) < -0.15), break; end
        if is_joint_in_collision(fk_new_g(2:7,:), voxel_grid, range), u1 = u1 + 1; break; end
        if is_robot_model_in_collision(fk_new_g(2:7,:), voxel_grid, range), u2 = u2 + 1; break; end

        % Přidání bodu do stromu
        Q_goal(:,end+1) = q_new_goal;
        P_goal(end+1) = idx_nearest_g;
        J_goal(:,:,end+1) = fk_new_g;

        idx_nearest_g = size(Q_goal,2);
        q_near = q_new_goal;

        % Kontrola spojení stromů
        if norm(q_new_goal - q_new_start) < goal_tolerance
            goal_idx_start = size(Q_start,2);
            goal_idx_goal = size(Q_goal,2);
            goal_flag = true;
            disp('Cíl dosažen pomocí RRT-Connect');
            break;
        end

    end

    if goal_flag, break; end

    % Výměna stromů pro další iteraci
    [Q_start, Q_goal] = deal(Q_goal, Q_start);
    [P_start, P_goal] = deal(P_goal, P_start);
    [J_start, J_goal] = deal(J_goal, J_start);
    tree_direction = -tree_direction;
end

% Pokud jsme nenalezli cestu do cíle -> stojíme na místě
if ~goal_flag
    q_path = [q_start, q_start];
    joint_path = repmat(forward_kinematics_ur5e(q_start), [2, 1, 1]);
    disp('Stojíme – cíl nenalezen.');
    return;
end

% Rekonstrukce výsledné cesty
idx_s = reconstruct_path_matrix(P_start, goal_idx_start);
idx_g = reconstruct_path_matrix(P_goal, goal_idx_goal);

% Ověření směru stromů
% Aby výsledná cesta byla seřazena od startu do cíle
if norm(Q_start(:,idx_s(1)) - q_start) > norm(Q_goal(:,idx_g(1)) - q_start)
    temp = Q_start; Q_start = Q_goal; Q_goal = temp;
    temp = P_start; P_start = P_goal; P_goal = temp;
    temp = J_start; J_start = J_goal; J_goal = temp;
    temp = idx_s; idx_s = idx_g; idx_g = temp;
end

% Sestavení konečné cesty
q_path = [Q_start(:,idx_s), fliplr(Q_goal(:,idx_g))];
joint_path = cat(3, J_start(:,:,idx_s), J_goal(:,:,fliplr(idx_g)));
joint_path = permute(joint_path, [3, 1, 2]);

end

%% Najde nejbližší bod ve stromu a vytvoří nový ve směru ke q_target

function [q_new, idx_nearest] = extend_tree_matrix(Q, q_target, step_size)

    % Vzdálenost všech bodů stromu k cíli    
    distances = vecnorm(Q - q_target, 2, 1);

    % Index nejbližšího
    [~, idx_nearest] = min(distances);
    q_near = Q(:,idx_nearest);

    % Vytvoření nového bodu ve směru
    direction = q_target - q_near;
    q_new = q_near + step_size * direction / (norm(direction) + 1e-6);
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