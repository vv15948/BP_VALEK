clc;
clear;
close all;
clear server

%% Inicializace voxelové mřížky

voxel_size = 0.05;
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

static_voxel_mask = generate_voxel_mask(range, box_params, cutout_params,voxel_size, voxel_grid);

voxel_grid = static_voxel_mask;

%% Nastavení TCP/IP server pro příjem dat z pythonu
port = 5051;
server = tcpserver("127.0.0.1", port, "Timeout", 10);
disp("MATLAB čeká na příjem dat");

%% Načtení robota
robot = loadrobot('universalUR5e', 'DataFormat', 'row');

%% Fronta pro přenos dat pro vizualizaci


%% První smyčka 

t_prev = tic; % pro signalizaci fps výpočtu

% Střídání předem definovaných startů a cílů
i = 1; 
while true

    [q_start, start_point, q_goal, goal_point, i] = get_next_waypoints(i);

    plot_counter = 0;
    plot_interval = 1;
    previous_path = q_start;
    ee_path = [];
    %% Hlavní smyčka
    while true

    %% Aktuální data ze serveru

    latest_data = "";
    while server.NumBytesAvailable > 0
        latest_data = readline(server);
    end

    % Dekódování z JSON do MATLAB struktury
    if latest_data ~= ""
        json_data = jsondecode(latest_data);
        if isempty(json_data.landmarks)
            fprintf("Nedostatek bodů v aktuálním snímku. Přeskakuji.\n");
            continue;
        end
        
    %% Vytvoření modelu člověka ve voxel_grid

        voxel_grid = fill_voxel_grid(voxel_grid, range, json_data.landmarks);

    %% Přidání start_point a goal_point do voxel_grid pro vizualizaci

        voxel_grid = add_start_and_goal(voxel_grid, range, start_point, goal_point);
%% 
% if is_path_valid(previous_path, voxel_grid,range, q_start)
%     q_path = previous_path    
% else
%     [q_path, ee_path] = rrt_6dof_basic(previous_path(1,:), q_goal, 2000, 0.3, voxel_grid, range);
% end
%% test
% --- VALIDACE CESTY ---
if size(previous_path, 1) == 1 && all(abs(previous_path(1,:) - q_start) < 1e-6)
    % Previous_path je jen q_start → není validní
    [q_path, ee_path] = rrt_6dof_basic(q_start, q_goal, 2000, 0.3, voxel_grid, range);
else
    if is_path_valid(previous_path, voxel_grid, range, q_start)
        q_path = previous_path;
    else
        [q_path, ee_path] = rrt_6dof_basic(previous_path(1,:), q_goal, 2000, 0.3, voxel_grid, range);
    end
end

% Ochrana proti selhání RRT
if isempty(q_path)
    warning("RRT selhalo nebo není platná cesta – přeskakuji iteraci");
    continue;
end
%%






%% FPS výpočtu
dt = toc(t_prev);
fps_compute = 1 / dt;

%% Plotování v každém x-tém kroku

plot_counter = plot_counter + 1;
if mod(plot_counter, plot_interval) == 0
    tic
    figure(1);  % nebo gcf, pokud nechceš přeskakovat
    
    plot_voxel_grid(voxel_grid, range, voxel_size,fps_compute,q_path(1,:),robot,ee_path);
    drawnow limitrate;  % důležité pro neblokující kreslení
toc
end
%% test
% if size(q_path,1) > 1
%     if size(q_path,1) == 2
%         previous_path = q_path(2,:);
%         ee_path = ee_path(2,:);
%         plot_voxel_grid(voxel_grid, range, voxel_size, fps_compute, previous_path, robot, ee_path);
%         break;
%     else
% previous_path = q_path(2:end,:);
% ee_path = ee_path(2:end,:);
%     end
%     break;
% end
%%
% --- POSUN PO CESTĚ ---
if size(q_path,1) > 1
    if size(q_path,1) == 2
        % Poslední krok – vykresli naposled a skonči
        previous_path = q_path(2,:);
        ee_path = ee_path(2,:);
        plot_voxel_grid(voxel_grid, range, voxel_size, fps_compute, previous_path, robot, ee_path);
        drawnow;
        break;  % ukonči vnitřní smyčku → vnější while načte nový waypoint
    else
        % Normální průchod – posuň se o krok
        previous_path = q_path(2:end,:);
        ee_path = ee_path(2:end,:);
    end

end
%%

%% FPS výpočtu
t_prev = tic;
    end
    voxel_grid(:) = static_voxel_mask;
    end
end

