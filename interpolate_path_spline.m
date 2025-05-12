function [q_path_spline, fk_path_spline] = interpolate_path_spline(q_path, N_interp)
% Zjemnění cesty pomocí spline v joint space

    % Interpolace v joint space
    t_original = linspace(0,1,size(q_path,2));
    t_fine = linspace(0,1,N_interp);

    % Inicializace
    q_path_spline = zeros(6, N_interp);

    % Spline funkce pro všechny klouby
    for j = 1:6
        q_path_spline(j,:) = spline(t_original, q_path(j,:), t_fine);
    end

    % Výpočet FK pro každý interpolovaný bod pro následné plotování TCP cesty
    fk_path_spline = zeros(N_interp, 7, 3);
    for i = 1:N_interp
        [fk_path_spline(i,:,:),~] = forward_kinematics_model(q_path_spline(:,i));
    end
end
