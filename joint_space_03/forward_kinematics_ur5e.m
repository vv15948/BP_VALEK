function positions = forward_kinematics_ur5e(q)
% Vrátí pozice všech 6 kloubů UR5e + TCP v prostoru
% q ... 1x6 vektor kloubových úhlů [rad]

% Délky článků UR5e (v metrech)
d1 = 0.1625;
a2 = -0.425;
a3 = -0.3922;
d4 = 0.1333;
d5 = 0.0997;
d6 = 0.0996;

% Přepočet jednotlivých transformačních matic podle DH parametrů
T = [-1     0     0     0;
     0     -1     0     0;
     0     0     1     0;
     0     0     0     1];
positions = zeros(7,3); % 6 kloubů + TCP

% DH parametry UR5e
DH = [...
    0,      pi/2,   d1,     q(1);
    a2,     0,      0,      q(2);
    a3,     0,      0,      q(3);
    0,      pi/2,   d4,     q(4);
    0,     -pi/2,   d5,     q(5);
    0,      0,      d6,     q(6)];

for i = 1:6
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);

    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0    ,          sin(alpha)   ,          cos(alpha)   ,     d      ;
              0    ,              0        ,              0        ,     1      ];

    T = T * A;
    positions(i,:) = T(1:3,4)';
end

% TCP pozice (koncový bod)
positions(7,:) = T(1:3,4)';
end