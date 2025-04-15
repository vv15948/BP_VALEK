clc;
clear;
close all;
clear server

%% Inicializace voxelové mřížky

voxel_size = 0.03; % Velikost jedné buňky [m]
[range, voxel_grid] = initializeVoxelGrid(voxel_size);
% voxel_grid - jen matice obsahující "0"

%% Doplnění statických překážek do voxel_grid

% Definice překážek - kvádrů
box_params = [-0.25,  0.75, -0.25,  0.25, -0.15, 0;
              -0.85,  -0.65, -0.55,  0.55, -0.15,  0.9];

% Definice výřezů - kvádrů
cutout_params = [-0.85,  -0.65, -0.45, -0.05, 0.45, 0.75;
                 -0.85,  -0.65, 0.05, 0.45, 0.45, 0.75;
                 -0.85,  -0.65, -0.45, -0.05, 0, 0.3;
                 -0.85,  -0.65, 0.05, 0.45, 0, 0.3];

% Vytvoření statických překážek a uložení do voxel_grid
static_voxel_mask = generate_voxel_mask(range, box_params, cutout_params,voxel_size, voxel_grid);
voxel_grid = static_voxel_mask;

%% Nastavení TCP/IP server pro příjem dat z pythonu

port = 5051;
server = tcpserver("127.0.0.1", port, "Timeout", 10);
disp("MATLAB čeká na příjem dat");

%% Načtení robota UR5e pro plot
robot = loadrobot('universalUR5e', 'DataFormat', 'row');


%% První smyčka 

% Střídání předem definovaných startů a cílů jako waypointů
way_point_index = 1; %Inicializace - začínáme na 1. bodu

while true

    [q_start, start_point, q_goal, goal_point, way_point_index] = get_next_waypoints(way_point_index);
        
    %% Přidání start_point a goal_point do voxel_grid pro vizualizaci
    
    init_voxel_grid = add_start_and_goal(voxel_grid, range, start_point, goal_point);
    
    % current_config = q_start;
    traveled_path = start_point;

    %% Výpočet prvotní cesty bez překážek

     [previous_q_path, previous_joint_path,u1,u2,goal_flag] = rrt_6dof_connect_03(q_start', q_goal', init_voxel_grid, range);

    % Vykreslení prvotní cesty ve statickém prostředí
    plot_voxel_grid(init_voxel_grid,range, voxel_size,previous_q_path(:,1)',robot, previous_joint_path(:,7,:),traveled_path)
    
    %% Hlavní smyčka
   
    while true
        
        % Grid obsahuje jen statické př. + start a cíl
        voxel_grid = init_voxel_grid; 

        %% Načtení dat z kamery

        latest_data = "";
        while server.NumBytesAvailable > 0
            latest_data = readline(server);
        end
    
        % Zpracování z JSON do MATLAB struktury
        if latest_data ~= ""
            json_data = jsondecode(latest_data);
            if isempty(json_data.landmarks)
                fprintf("Nedostatek bodů v aktuálním snímku. Přeskakuji.\n");
                continue;
            end
        
        %% Vytvoření modelu člověka ve voxel_grid

        voxel_grid = fill_voxel_grid(voxel_grid, range, json_data.landmarks);
        end

        %% Validace předchozí cesty

        [is_valid, collision_index] = is_path_valid(previous_q_path, previous_joint_path, voxel_grid, range);

        if is_valid
            % Cesta je platná 
            q_path = previous_q_path;
            joint_path = previous_joint_path;
            traveled_path(end+1,:) = squeeze(previous_joint_path(1,7,:)); % Ukládání projeté cesty
            
            plot_voxel_grid(voxel_grid,range, voxel_size,q_path(:,1)',robot, joint_path(1:end,7,:),traveled_path)
            pause(0.01)
        else
            % Cesta koliduje - přepočet cesty
            [q_path, joint_path,u1,u2,goal_flag] = rrt_6dof_connect_03(previous_q_path(:,1), q_goal', voxel_grid, range);
    
            % Pokud se podařilo najít cestu - ukládáme a vizualizujeme
            if goal_flag
                traveled_path(end+1,:) = squeeze(joint_path(1,7,:));
                plot_voxel_grid(voxel_grid,range, voxel_size,q_path(:,1)',robot, joint_path(:,7,:),traveled_path)
                pause(0.01)
            else
            % Pokud ne - stojíme na místě
                plot_voxel_grid(voxel_grid,range, voxel_size,previous_q_path(:,1)',robot, previous_joint_path(:,7,:),traveled_path)  
                pause(0.01)
                continue;  
            end
        end
    
        % Pokud jsme na konci cesty opusíme smyčku -> změna waypointů
        if size(q_path, 2) <= 1
            disp("Cesta dokončena.");
        break;
        end

    % V každé iteraci posun o 1 konfiguraci aktuální cesty
    previous_q_path = q_path(:,2:end);
    previous_joint_path = joint_path(2:end,:,:);
    end
end
