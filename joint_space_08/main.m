clc;
clear;
close all;
clear server

%% Inicializace voxelové mřížky

voxel_size = 0.05; % Velikost jedné buňky [m]
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
static_voxel_grid = static_voxel_mask;

%% Nastavení TCP/IP server pro příjem dat z pythonu

port = 5051;
server = tcpserver("127.0.0.1", port, "Timeout", 10);
disp("MATLAB čeká na příjem dat");

%% Načtení robota UR5e pro plot
robot = loadrobot('universalUR5e', 'DataFormat', 'row');


%% První smyčka 

% Střídání předem definovaných startů a cílů jako waypointů
way_point_index = 2; %Inicializace - začínáme na 1. bodu

while true

     [q_start, start_point, q_goal, goal_point, way_point_index] = get_next_waypoints(way_point_index);

    %% Přidání start_point a goal_point do voxel_grid pro vizualizaci
    
    init_voxel_grid = add_start_and_goal(static_voxel_grid, range, start_point, goal_point);
    


    %% Výpočet prvotní cesty bez překážek
    
     % [previous_q_path, previous_joint_path,u1,u2,goal_flag] = rrt_6dof_connect_03(q_start', q_goal', init_voxel_grid, range);
     [previous_q_path, previous_joint_path,u1,u2,goal_flag] = rrt_6dof_connect_02(q_start', q_goal', init_voxel_grid, range);
     
   

    % Vykreslení prvotní cesty ve statickém prostředí
    traveled_path = start_point;
    plot_voxel_grid(init_voxel_grid,range, voxel_size,previous_q_path(:,1)',robot, previous_joint_path(:,5,:),previous_joint_path(1:end,4,:),traveled_path)
    pause(0.5)
    %% Hlavní smyčka
   
    while true
        
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
        voxel_grid = init_voxel_grid; % Se statickými překážkami
        voxel_grid = fill_voxel_grid(voxel_grid, range, json_data.landmarks);
        

        else
            % Pouze statické překážky
            voxel_grid = init_voxel_grid;
        end
         % voxel_grid = fill_sphere(voxel_grid, range(1,:), range(2,:), range(3,:), [0.1 0.1 0.8], 0.2);
        %% Validace předchozí cesty
        
        [is_valid, collision_index] = is_path_valid(previous_q_path, previous_joint_path(:,1:4,:),voxel_grid,range);

        collision_index;
        if is_valid
            % Cesta je platná 
            q_path = previous_q_path;
            joint_path = previous_joint_path;
            traveled_path(end+1,:) = squeeze(previous_joint_path(1,5,:)); % Ukládání projeté cesty
            
            plot_voxel_grid(voxel_grid,range, voxel_size,q_path(:,1)',robot, joint_path(1:end,5,:), joint_path(1:end,4,:),traveled_path)
            pause(0.1)
        else
            tic
            % Cesta koliduje - přepočet cesty
             [q_path, joint_path,u1,u2,goal_flag] = rrt_6dof_connect_02(previous_q_path(:,1), q_goal', voxel_grid, range);
           u1 % kolik bodu neproslo prvni fazi kolize
           u2 % druha faze kolize
             toc
            

            % Pokud se podařilo najít cestu - ukládáme a vizualizujeme
            if goal_flag
                traveled_path(end+1,:) = squeeze(joint_path(1,5,:));
                plot_voxel_grid(voxel_grid,range, voxel_size,q_path(:,1)',robot, joint_path(:,5,:),joint_path(1:end,4,:),traveled_path)
                pause(0.1)
            else
            % Pokud ne - stojíme na místě
                plot_voxel_grid(voxel_grid,range, voxel_size,previous_q_path(:,1)',robot, previous_joint_path(:,5,:),previous_joint_path(:,4,:),traveled_path)  
                pause(0.1)
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
