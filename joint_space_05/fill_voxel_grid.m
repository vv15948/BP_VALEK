function voxel_grid = fill_voxel_grid(voxel_grid, range, landmarks)
    
x_range = range(1,:);
y_range = range(2,:);
z_range = range(3,:);

%% Hlava a zápěstí
% Definování poloměrů pro hlavu a zápěstí
    head_radius = 0.2;
    wrist_radius = 0.15;

    % Přidání sféry kolem hlavy
    if isfield(landmarks, 'head')
        voxel_grid = fill_sphere(voxel_grid, x_range, y_range, z_range, ...
                                 [landmarks.head.x, landmarks.head.y, landmarks.head.z], ...
                                 head_radius);
    end

    % Přidání sféry kolem zápěstí
    for side = ["left", "right"]
        wrist_key = sprintf('wrist_%s', side);
        if isfield(landmarks, wrist_key)
            voxel_grid = fill_sphere(voxel_grid, x_range, y_range, z_range, ...
                                     [landmarks.(wrist_key).x, landmarks.(wrist_key).y, landmarks.(wrist_key).z], ...
                                     wrist_radius);
        end
    end

%% Ruce a nohy

    % Definování propojení mezi klíčovými body (pro válce)
    connections = {
        "shoulder_left", "elbow_left", 0.1;  % Levá paže
        "elbow_left", "wrist_left", 0.08;      % Levé předloktí
        "shoulder_right", "elbow_right", 0.1;% Pravá paže
        "elbow_right", "wrist_right", 0.08;    % Pravé předloktí
        "hip_left", "knee_left", 0.15;         % Levé stehno
        "hip_right", "knee_right", 0.15;       % Pravé stehno
    };   
    
    % Přidání válců mezi klíčovými body
    for i = 1:size(connections, 1)
        point1_name = connections{i, 1};
        point2_name = connections{i, 2};
        radius = connections{i, 3};

        if isfield(landmarks, point1_name) && isfield(landmarks, point2_name)
            start_point = [landmarks.(point1_name).x, landmarks.(point1_name).y, landmarks.(point1_name).z];
            end_point = [landmarks.(point2_name).x, landmarks.(point2_name).y, landmarks.(point2_name).z];

            voxel_grid = add_cylinder_to_voxel_grid(voxel_grid, x_range, y_range, z_range, start_point, end_point, radius);
        end
    end

%% Trup
    
    % radius pro trup
    body_radius = 0.2;

    % Výpočet středů ramen a kyčlí
    shoulder_mid = [];
    hip_mid = [];

    if isfield(landmarks, 'shoulder_left') && isfield(landmarks, 'shoulder_right')
        shoulder_mid = average_point(landmarks.shoulder_left, landmarks.shoulder_right);
    end
    
    if isfield(landmarks, 'hip_left') && isfield(landmarks, 'hip_right')
        hip_mid = average_point(landmarks.hip_left, landmarks.hip_right);
    end
    
    % Přidání válce pro trup
    if ~isempty(shoulder_mid) && ~isempty(hip_mid)
        voxel_grid = add_cylinder_to_voxel_grid(voxel_grid, x_range, y_range, z_range, ...
            shoulder_mid, hip_mid, body_radius);
    end

end

function result = average_point(p1, p2)
    result = ([p1.x, p1.y, p1.z] + [p2.x, p2.y, p2.z]) / 2;
end