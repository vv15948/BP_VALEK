function [q_path, joint_path, u1, u2, goal_flag,dirout] = rrt_6dof_connect_02(q_start, q_goal, voxel_grid, range)



%% inicializace

u1 = 0; u2 = 0;     % kolik konfigurací neprošlo kolizními kontrolami
goal_flag = false;  % Zda došlo k nalezení cesty
goal_idx_start = 0; % Indexy propojení obou stromů
goal_idx_goal = 0;


%% Nastavení parametrů

max_step_per_joint = [0.25; 0.15; 0.12; 0.1; 0.1; 0.1]*1.2;
max_iter = 3000;                                % Max. počet iterací                                        
goal_tolerance = 0.2;                           % Vzdálenost od cíle, kdy považujeme, že už jsme v cíli
joint_limits = repmat([-2*pi, 2*pi], 6, 1);     % Fyzické omezení kloubových rozsahů robota UR5e
goal_bias = 0.5;                                % Jak často se budou generovat nové body přímo do cílové konfigurace
max_s = 3;   % Kolikrát se pokusíme udělat v 1 iteraci kroků od 2. stromu k 1.

%% Inicializace stromů

% Počáteční nastavení stromů ze startu a z cíle
Q_start = q_start;
[fk_start(:,:,1), dir_start(1,:)] = forward_kinematics_model(q_start);
P_start = 0;
Q_goal = q_goal;
[fk_goal(:,:,1), dir_goal(1,:)] = forward_kinematics_model(q_goal);
P_goal = 0;

% Počáteční růst (1 = strom ze startu)
tree_direction = 1;

%% Hlavní smyčka 

for iter = 1:max_iter

    % Výběr náhodného bodu/cílového podle nastavení goal_bias v rozsahu robota
    if rand < goal_bias
        q_rand = (tree_direction == 1) * Q_goal(:,1) + (tree_direction == -1) * Q_start(:,1);
    else
        q_rand = joint_limits(:,1) + (joint_limits(:,2) - joint_limits(:,1)) .* rand(6,1);
    end

    % Rozšíření stromu ve směru ke q_rand
    [q_new_start, idx_nearest_s] = extend_tree_matrix(Q_start, q_rand, max_step_per_joint);
    if isempty(q_new_start), continue; end

    %% Kolizní kontrola 

    % Přímá kinematika pro vypočtenou konfiguraci
    [fk_pos_s,dir] = forward_kinematics_model(q_new_start);

    % Pokud některý kloub leží pod základnou robota, tak konfiguraci zahazuji
    if any(fk_pos_s(:,3) < 0), continue; end

    % Pokud poloha některého z kloubů leží v překážce, tak taky konfig. zahazuji
    if is_joint_in_collision(fk_pos_s, voxel_grid, range), u1 = u1 + 1; continue; end

    % Kolizni kontrola pomocí modelu robota z válců
    if is_robot_model_in_collision(fk_pos_s, voxel_grid, range,dir), u2 = u2 + 1; continue; end

    %% Pokud konfigurace prošla všemi kontrolami, tak ji přidáme do stromu

    Q_start(:,end+1) = q_new_start;
    P_start(end+1) = idx_nearest_s;
    dir_start(end+1,:) = dir;
    fk_start(:,:,end+1) = fk_pos_s;


    %% Vyhledání njebližšího bodu ve druhém stromě

    distances = vecnorm(Q_goal - q_new_start, 2, 1);
    [~, idx_nearest_g] = min(distances);
    q_near = Q_goal(:,idx_nearest_g);

    % Rozšiřování druhého stromu
    s=0;
    while true

        % Výpočet směru a vzdálenosti mezi těmito 2 konfiguracemi
        direction = q_new_start - q_near;
        dist = norm(direction);
        if dist < 1e-6, break; end

        % Vytvoření nové konfigurace v daném směru
        q_new_goal = q_near + max_step_per_joint .* direction / dist;

        % Ověření, zda se q_new_goal přibližuje ke q_goal (nevrací se zpět)
if norm(q_new_goal - q_goal) > norm(q_near - q_goal)+0.2
    break;  % pokračování by vedlo od cíle, ukončíme rozšiřování
end
        % Kolizní kontrola
        [fk_pos_g,dir] = forward_kinematics_model(q_new_goal);
        if any(fk_pos_g(:,3) < 0), break; end
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
    joint_path = zeros(2,7,3);
    joint_path(1,:,:)=fk_start(:,:,1);
    joint_path(2,:,:)=fk_start(:,:,1);
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

%% Interpolace mezi poslednimi 2 body, aby byla sance dojet do cile

% ulozeni koncových uzlů
q_path_end = q_path(:,end);
dirout_end = dirout(end,:);
fk_combined_end = fk_combined(:,:,end);

% Počet interpolací
N = 2;

% Poslední 2 body
q1 = q_path(:,end-1);
q2 = q_path(:,end);

% Inicializace
q_interp = zeros(6,N);
fk_interp = zeros(7,3,N);
dir_interp = zeros(N,3);

% Nalezení interpolovaných bodů a doplnění výstupu z fk funkce
for i=1:N
    alpha = i/(N+1);
    q_interp(:,i) = (1- alpha)*q1 + alpha*q2;
    [fk_interp(:,:,i),dir_interp(i,:)] = forward_kinematics_model(q_interp(:,i));
end

% doplnění do výstupu
q_path(:,end:end+N-1) = q_interp;
dirout(end:end+N-1,:) = dir_interp;
fk_combined(:,:,end:end+N-1) = fk_interp;

q_path(:,end+1) = q_path_end;
dirout(end+1,:) = dirout_end;
fk_combined(:,:,end+1) = fk_combined_end;

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

