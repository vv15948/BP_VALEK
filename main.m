clc;
clear;
close all;
clear server

%% Inicializace

    %% Vytvoření složky pro ukládání dat
save_folder = 'snapshot_data';
if exist(save_folder, 'dir')
    rmdir(save_folder, 's');
end
mkdir(save_folder);
frame_index = 1;

    %% Připojení k robotu

IP = '10.1.1.2' % IP address of the robot
ur = UR_robot('robot','ip',IP,'user','admin','password','KPivuKlobaskaKVeceruProchazka');

ur.r = 0.04;        % Nastavení blending raidiusu. Jak moc robot nemusí projet konkrétními konfiguracemi
ur.a_joint = 0.25;  % Max zrychlení 
ur.v_joint = 0.3;   % Max rychlost

    %% Voxel_grid

% Velikost jedné buňky [m]
voxel_size = 0.05;  

% range - zarovnání mřížky podle souřadnic
% voxel_grid - základní mřížka (matice) obsahující "0"
[range, voxel_grid] = initializeVoxelGrid(voxel_size);

    %% Statické překážky podle CAD

% Definice překážek - kvádrů
box_params = [-0.25,  0.25, -0.22,  0.8, -0.15, 0;
              -0.6,  0.6, -0.7,  -0.55, -0.15,  0.9];

% Definice výřezů - kvádrů
cutout_params = [-0.45, -0.05, -0.85,  -0.55, 0.45, 0.75;
                 0.05, 0.45, -0.85,  -0.55, 0.45, 0.75;
                 -0.45, -0.05, -0.85,  -0.55, 0, 0.3;
                 0.05, 0.45, -0.85,  -0.55, 0, 0.3];

% static_voxel_grid - voxel_grid obsahující statické překážky
static_voxel_grid = generate_voxel_mask(range, box_params, cutout_params,voxel_size, voxel_grid);

    %% Nastavení TCP/IP server pro příjem dat z pythonu

port = 5051;
server = tcpserver("127.0.0.1", port, "Timeout", 10);
disp("MATLAB čeká na příjem dat");
while server.NumBytesAvailable == 0, pause(0.1); end
disp("kamera pripojena")


%% První smyčka 

%% Střídání předem definovaných startů a cílů jako waypointů

% Inicializace - začínáme na 1. bodu
way_point_index = 1; 

while true

    % Definice waypointů [q1 - q6]
    waypoints =[-1.339361492787500  -2.251367231408590  -2.077919244766240   1.190111561412480   1.348566412925720   1.570490598678590;
                -4.787263909970420  -1.988374372521870  -1.188712000846860  -0.991601244812348   1.480629205703740   1.590799450874330];

    % Podle aktuálního way_point_index - výběr:
    % q_start a q_goal z waypoints
    % start_point a goal_point - poloha TCP v prostoru pro startovní a cílovou konfiguraci (pro plot)
    % way_point_index - pro další smyčku už je definovaný ve funkci
    [q_start, start_point, q_goal, goal_point, way_point_index] = get_next_waypoints(waypoints,way_point_index);

    %% Přidání start_point a goal_point do voxel_grid pro vizualizaci
    
    voxel_grid_start = add_start_and_goal(static_voxel_grid, range, start_point, goal_point);
    
    %% Výpočet prvotní cesty v prostředí se statickými překážkami

    
    [actual_q_path, actual_joint_position_path,u1,u2,goal_flag,actual_dir] = rrt_6dof_connect_02(q_start', q_goal', voxel_grid_start, range);
    

    % Vyhlazení výsledné cesty pomocí spline funkce
    N_interp = 150;
    [actual_q_path_spline, actual_joint_position_path_spline] = interpolate_path_spline(actual_q_path, N_interp);
    
    % Počkám, až robot dojede do startovní konfig.
    ur.wait_mode = 'on';
    ur.set_config(q_start')

    % Nyní se už pohybuje podle vypočítané cesty
    ur.wait_mode = 'off';
    ur.set_config(actual_q_path_spline)



    %% Hlavní smyčka - validace cesty (popřípadě přeplánování) + posun po cestě až do cíle

    while true
        
        %% Načtení dat z kamery

        latest_data = "";
        while server.NumBytesAvailable > 0
            latest_data = readline(server);
        end
    
        % Zpracování z JSON do MATLAB struktury
        if latest_data ~= ""
            json_data = jsondecode(latest_data);
            t_matlab = posixtime(datetime('now','TimeZone','UTC')); % Uložení času, kdy přišly data z kamery
            if isempty(json_data.landmarks)
                fprintf("Nedostatek bodů v aktuálním snímku. Přeskakuji.\n");
                continue;
            end
        
        %% Vytvoření modelu člověka ve voxel_grid

        voxel_grid = fill_voxel_grid(voxel_grid_start, range, json_data.landmarks);
        
        % Pokud z kamery nepřišla žádná data, tak voxel_grid bude obsahovat pouze statické překážky
        else
            voxel_grid = voxel_grid_start;
        end

        %% Validace aktuální cesty
        
        [is_valid, collision_index] = is_path_valid(actual_q_path, actual_joint_position_path,voxel_grid,range,actual_dir);
        
        % Pokud jsme na konci cesty opusíme smyčku -> změna waypointů    
        if norm(actual_q_path(:,1)'- q_goal) < 0.3
            disp("Cesta dokončena.");     
        break;
        end   

        % Pokud nezasahuje do překážek, tak jede podél aktuální cesty
        if is_valid

        %% Uložení dat
        
        t = posixtime(datetime('now','TimeZone','UTC'));
        delay = t - t_matlab;    % Čas kolik zabralo zpracování dat ro následnou synchronizaci videa
        timestamp = json_data.time + delay;
        q_now = ur.config';
        filename = fullfile(save_folder, sprintf('snapshot_%03d.mat', frame_index));
        save(filename, 'voxel_grid','range','voxel_size','q_now', 'actual_q_path','actual_joint_position_path_spline','traveled_path','timestamp'); 
        frame_index = frame_index + 1;
        %%


        % Pokud cesta vede přes překážku, tak ji musíme přepočítat
        else
            
            % Počáteční konfigurace pro plánování (informace přímo z robota)          
            q_now = ur.config;
            
            % Příkaz k zastavení robota 
            ur.stop; 

            % Přeplánování cesty         
            [actual_q_path, actual_joint_position_path,u1,u2,goal_flag,actual_dir] = rrt_6dof_connect_02(q_now, q_goal', voxel_grid, range);                      

            % Pokud jsme našli cestu
            if goal_flag

                % Vyhlazení pomocí spline funkce             
                [actual_q_path_spline, actual_joint_position_path_spline] = interpolate_path_spline(actual_q_path, N_interp);
                
                % Poslání vypočítané cesty robotu
                ur.set_config(actual_q_path_spline)

                %% Uložení dat
                
                t = posixtime(datetime('now','TimeZone','UTC'));
                delay = t - t_matlab;    % Čas kolik zabralo zpracování dat ro následnou synchronizaci videa
                timestamp = json_data.time + delay;
                q_now = ur.config';
                filename = fullfile(save_folder, sprintf('snapshot_%03d.mat', frame_index));
                save(filename, 'voxel_grid','range','voxel_size','q_now', 'actual_q_path','actual_joint_position_path_spline','traveled_path','timestamp'); 
                frame_index = frame_index + 1;
                %%
                
            % Pokud jsme nenašli cestu
            else
                
                % Dochází k zastavení robota
                ur.set_config(actual_q_path); % Odpovídá pouze aktuální konfiguraci, takže se robot nikam nepohybuje
                    
                actual_q_path_spline = actual_q_path;
                actual_joint_position_path_spline = actual_joint_position_path;

                %% Uložení dat
                
                t = posixtime(datetime('now','TimeZone','UTC'));
                delay = t - t_matlab;    % Čas kolik zabralo zpracování dat ro následnou synchronizaci videa
                timestamp = json_data.time + delay;
                q_now = ur.config';
                filename = fullfile(save_folder, sprintf('snapshot_%03d.mat', frame_index));
                save(filename, 'voxel_grid','range','voxel_size','q_now', 'actual_q_path','actual_joint_position_path_spline','traveled_path','timestamp'); 
                frame_index = frame_index + 1;
                %%
                
                continue;  
            end
        end
     
        %% Posun po cestě na konfiguraci, kde se robot právě nachází
        q_now = ur.config;                          % Aktuální konfigurace přímo z robota    
        dist = vecnorm(actual_q_path-q_now,2,1);    
        [a,idx_cl]=min(dist);                       % idx_cl - index nejbližšího bodu na aktuální cestě

        % Zkrácení cesty od nejbližší konfigurace k robotovi do cíle
        actual_q_path = actual_q_path(:,idx_cl:end);   
        actual_joint_position_path = actual_joint_position_path(idx_cl:end,:,:);
        actual_dir = actual_dir(idx_cl:end,:);
        
        % Ořezání i zjemněné cesty od daného indexu
        dist = vecnorm(actual_q_path_spline - actual_q_path(:,1),2,1);
        [~, idx_match] = min(dist);
        actual_q_path_spline = actual_q_path_spline(:,idx_match:end);
        actual_joint_position_path_spline = actual_joint_position_path_spline(idx_match:end,:,:);
    end
end

