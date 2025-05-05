clc;
clear;
close all;
clear server

%% Object creation
IP = '10.1.1.2' % IP address of the robot
ur = UR_robot('robot','ip',IP,'user','admin','password','KPivuKlobaskaKVeceruProchazka');

ur.r = 0.05;
ur.a_joint = 0.1;
ur.v_joint = 0.35;

k =0;

%% Inicializace voxelové mřížky

voxel_size = 0.05; % Velikost jedné buňky [m]
[range, voxel_grid] = initializeVoxelGrid(voxel_size);
% voxel_grid - jen matice obsahující "0"

%% Doplnění statických překážek do voxel_grid

% Definice překážek - kvádrů
box_params = [-0.25,  0.25, -0.22,  0.8, -0.15, 0;
              -0.6,  0.6, -0.7,  -0.55, -0.15,  0.9];

% Definice výřezů - kvádrů

cutout_params = [-0.45, -0.05, -0.85,  -0.55, 0.45, 0.75;
                 0.05, 0.45, -0.85,  -0.55, 0.45, 0.75;
                 -0.45, -0.05, -0.85,  -0.55, 0, 0.3;
                 0.05, 0.45, -0.85,  -0.55, 0, 0.3];

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
  tic
     [previous_q_path, previous_joint_path,u1,u2,goal_flag,previous_dir] = rrt_6dof_connect_02(q_start', q_goal', init_voxel_grid, range);
  
     toc
     % Vykreslení prvotní cesty ve statickém prostředí
    traveled_path = start_point;
     plot_voxel_grid(init_voxel_grid,range, voxel_size,previous_q_path(:,1)',robot, previous_joint_path(:,7,:),previous_joint_path(1:end,4,:),traveled_path)
     
     % Počkám, až robot dojede do startovní konfig.
    ur.wait_mode = 'on';
     ur.set_config(q_start')
     % Nyní se už pohybuje podle vypočítané cesty
   ur.wait_mode = 'off';
    ur.set_config(previous_q_path)

    
    pause(0.1)
    %% Hlavní smyčka
   i=0;
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
        % if i>=5
        %     voxel_grid = fill_sphere(voxel_grid, range(1,:), range(2,:), range(3,:), [0.6 0 0.5], 0.2);
        % end
        %% Validace předchozí cesty
        
        [is_valid, collision_index] = is_path_valid(previous_q_path, previous_joint_path,voxel_grid,range,previous_dir);

        collision_index;
        if is_valid
            % Cesta je platná 
            q_path = previous_q_path;
            joint_path = previous_joint_path;
            dir = previous_dir;
            traveled_path(end+1,:) = squeeze(previous_joint_path(1,5,:)); % Ukládání projeté cesty
            
            plot_voxel_grid(voxel_grid,range, voxel_size,q_path(:,1)',robot, joint_path(:,7,:), joint_path(:,4,:),traveled_path)
            pause(0.1)
        else
            tic
            % Cesta koliduje - přepočet cesty
            q_actual = ur.config;
            dist = vecnorm(q_path-q_actual,2,1);
            [a,idx_cl]=min(dist);
             [q_path, joint_path,u1,u2,goal_flag,dir] = rrt_6dof_connect_02(q_path(:,idx_cl+1), q_goal', voxel_grid, range);
           u1 % kolik bodu neproslo prvni fazi kolize
           u2 % druha faze kolize
             toc
             % posun na začátek nově vypočítané cesty (mělo by být tam, kde
             % se robot nachází
            ur.wait_mode = 'on';
            ur.set_config(q_path(:,1))
            % Nyní se pohybuje podle přepočítané cesty
            ur.wait_mode = 'off';
            ur.set_config(q_path)

            % Pokud se podařilo najít cestu - ukládáme a vizualizujeme
            if goal_flag
                traveled_path(end+1,:) = squeeze(joint_path(1,5,:));
                plot_voxel_grid(voxel_grid,range, voxel_size,q_path(:,1)',robot, joint_path(:,7,:),joint_path(:,4,:),traveled_path)
                pause(0.1)
            else
            % Pokud ne - stojíme na místě
                plot_voxel_grid(voxel_grid,range, voxel_size,previous_q_path(:,1)',robot, previous_joint_path(:,7,:),previous_joint_path(:,4,:),traveled_path)  
                pause(0.1)
                continue;  
            end
        end
    
        % Pokud jsme na konci cesty opusíme smyčku -> změna waypointů
        if size(q_path, 2) <= 2
            disp("Cesta dokončena.");

        break;
        end
        % ořežeme cestu od uzlu, ke kteremu je robot aktualně nejblíž
    q_actual = ur.config;
    dist = vecnorm(q_path-q_actual,2,1);
    [a,idx_cl]=min(dist);
    if idx_cl == 1
        k=k+1;
    else
        k=0;
    end
    if k >= 3;
        ur.wait_mode = 'on';
        ur.set_config(q_path(:,2));
        ur.wait_mode = 'off';
        ur.set_config(q_path(:,2:end));
        idx_cl = 2;
        k=0;
        disp('cekam')
    end
    
    previous_q_path = q_path(:,idx_cl:end);
    previous_joint_path = joint_path(idx_cl:end,:,:);
    previous_dir = dir(idx_cl:end,:);
   
    i=i+1;
    end
    pause(1)
end


