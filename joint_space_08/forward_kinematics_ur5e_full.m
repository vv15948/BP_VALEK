function positions = forward_kinematics_ur5e_full(q)
% Vrátí pozice všech 6 kloubů UR5e + TCP v prostoru
% q ... 1x6 vektor kloubových úhlů [rad]


% Délka nástroje na koncovém efektoru
tool_length = 0;

% Délky článků UR5e (v metrech)
d1 = 0.1625;
a2 = -0.425;
a3 = -0.3922;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996 + tool_length;

% Inicializační transpormační matice
T = [-1     0     0     0;
     0     -1     0     0;
     0     0     1     0;
     0     0     0     1];

% DH parametry UR5e
DH = [...
    0,      pi/2,   d1,     q(1);
    a2,     0,      0,      q(2);
    a3,     0,      0,      q(3);
    0,      pi/2,   d4,     q(4);
    0,     -pi/2,   d5,     q(5);
    0,      0,      d6,     q(6)];

% Inicializace výstupu
% 6 kloubů + TCP
positions = zeros(7,3); 

% Výpočet dopředné kinematiky
for i = 1:6
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);

    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0    ,          sin(alpha)   ,          cos(alpha)   ,     d      ;
              0    ,              0        ,              0        ,     1      ];

    T = T * A;
    % Uložení poloh kloubů J1-J6
    positions(i,:) = T(1:3,4)';
end

% TCP pozice (koncový bod)
positions(7,:) = T(1:3,4)';
end

