clear; clc;

syms q1 q2 q3 q4 q5 q6

% 6 DoF homogeneous transformation matrices
T01 = TZ(-q1,[0; 0; 0.1564]) * ...
      [1  0  0 0
       0 -1  0 0
       0  0 -1 0
       0  0  0 1];
   
T12 = TY(-q2,[0; 0.0054; -0.128]) * ...
      [1 0 0 0
       0 0 -1 0
       0 1  0 0
       0 0  0 1];
   
T02 = simplify(T01*T12);
   
T23 = TZ(-q3,[0; -0.410; 0]) * ...
      [1  0  0   0
       0 -1  0   0
       0  0  -1  0
       0  0  0   1];
T03 = simplify(T01*T12*T23);
   
T34 = TY(-q4,[0; 0.2084; -0.0064]) * ...
      [1 0 0  0
       0 0 -1 0
       0 1 0  0
       0 0 0  1];
   T04 = simplify(T01*T12*T23*T34);
   
T45 = TY(q5,[0; 0; -0.1059]) * ...
      [1 0  0 0
       0 0  1 0
       0 -1 0 0
       0 0  0 1];
   
   T05 = simplify(T01*T12*T23*T34*T45);

   
T56 = TY(-q6,[0; 0.1059; 0]) * ...
      [1 0 0  0
       0 0 -1 0
       0 1 0  0
       0 0 0  1];
T06 = simplify(T01*T12*T23*T34*T45*T56);
 
T67 = TZ(-0,[0; 0; -0.0615]) * ...
      [-1 0 0 0
       0 1 0  0
       0 0 -1 0
       0 0  0 1];
T = simplify(T01*T12*T23*T34*T45*T56*T67);
   
 % Motion axes
Z0 = [0; 0; 1];
Z1 = T01(1:3,3);
Z2 = T02(1:3,3);
Z3 = T03(1:3,3);
Z4 = T04(1:3,3);
Z5 = T05(1:3,3);
 
% Position vectors 
P0 = [0; 0; 0];
P1 = T01(1:3,4);
P2 = T02(1:3,4);
P3 = T03(1:3,4);
P4 = T04(1:3,4);
P5 = T05(1:3,4);
Pe = T(1:3,4);

% Linear velocity jacobians
Jv1 = Z0;
Jv2 = cross(Z1,Pe-P1);
Jv3 = cross(Z2,Pe-P2);
Jv4 = Z3;
Jv5 = cross(Z4,Pe-P4);
Jv6 = cross(Z5,Pe-P5);
 
% Angular velocity Jacobians
Jw1 = [0; 0; 0];
Jw2 = Z1;
Jw3 = Z2;
Jw4 = [0; 0; 0];
Jw5 = Z4;
Jw6 = Z5;

% 6x6 Jacobian
J = simplify([Jv1 Jv2 Jv3 Jv4 Jv5 Jv6
              Jw1 Jw2 Jw3 Jw4 Jw5 Jw6])



function T = TX(q, p)
    T = [1     0       0     p(1)
         0   cos(q)  -sin(q) p(2)
         0   sin(q)   cos(q) p(3)
         0     0        0     1];
end

function T = TY(q, p)
    T = [cos(q)  0 sin(q) p(1)
           0     1  0     p(2)
         -sin(q) 0 cos(q) p(3)
           0     0  0      1];
end

function T = TZ(q, p)
    T = [cos(q) -sin(q) 0 p(1)
         sin(q)  cos(q) 0 p(2)
            0     0     1 p(3)
            0     0     0 1];
end