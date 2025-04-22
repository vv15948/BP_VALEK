function [positions] = forward_kinematics_ur5e(q)
% Verze s 4 DOF vstupem
% q ... 1x4 vektor kloubových úhlů (q1–q4)
% Výstup:
%   positions - pozice kloubů J1–J4 (4x3)
%   T_end     - transformace posledního kloubu (4x4)

% Délky článků UR5e (v metrech)
d1 = 0.1625;
a2 = -0.425;
a3 = -0.3922;
d4 = 0.1333;

% Inicializační transformační matice
T = [-1     0     0     0;
     0     -1     0     0;
     0     0     1     0;
     0     0     0     1];

% DH parametry pouze pro q1–q4
DH = [...
    0,      pi/2,   d1,     q(1);
    a2,     0,      0,      q(2);
    a3,     0,      0,      q(3);
    0,      pi/2,   d4,     q(4)];

positions = zeros(4,3);  % pozice J1–J4

for i = 1:4
    a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4);

    A = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0    ,          sin(alpha)   ,          cos(alpha)   ,     d      ;
              0    ,              0        ,              0        ,     1      ];

    T = T * A;
    positions(i,:) = T(1:3,4)';
end

end


