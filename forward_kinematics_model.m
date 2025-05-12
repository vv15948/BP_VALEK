function [positions, directions] = forward_kinematics_model(q)
% Funkce pro výpočet přímé kinematiky z dané kloubové konfigurace q

    tool = 0;
    
    % Délky článků UR5e (v metrech)
    d1 = 0.1625;
    a2 = -0.425;
    a3 = -0.3922;
    d4 = 0.1333;
    d5 = 0.0997;
    d6 = 0.0996 + tool;
    
    % Inicializační transformační matice (počáteční orientace)
    T = [-1 0 0 0;
          0 -1 0 0;
          0  0 1 0;
          0  0 0 1];
    
    % DH parametry UR5e
    DH = [0,      pi/2,   d1,     q(1);
          a2,     0,      0,      q(2);
          a3,     0,      0,      q(3);
          0,      pi/2,   d4,     q(4);
          0,     -pi/2,   d5,     q(5);
          0,      0,      d6,     q(6)];
    
    % Inicializace výstupů
    positions = zeros(7,3);     % 6 kloubů + TCP
    directions = zeros(1,3);    % směry po kloubech 1–3
    
    % Výpočet FK
    for i = 1:6
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);
    
        A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                  0    ,          sin(alpha)   ,          cos(alpha)   ,     d      ;
                  0    ,              0        ,              0        ,     1      ];
    
        T = T * A;
        
        % ukládání polohy jednotlivých kloubů
        positions(i,:) = T(1:3,4)';
        
        % Pro první kloub uložení směrového vektoru pro kolizní kontrolu (offset 1. ramene)
        if i == 1
            x_axis_global = T(1:3,1);
            directions(i,:) = x_axis_global / norm(x_axis_global);
        end
    end
    
    % TCP pozice (stejná jako 6. kloub, protože tool = 0)
    positions(7,:) = T(1:3,4)';
end
