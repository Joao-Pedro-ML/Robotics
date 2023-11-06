%% Exercício 1
%   theta | d | a | alfa | sigma
% 1   q1  | 50| 0 |   0  |  R=0
% 2   q2  | 0 | 0 |  -90 |  R=0
% 3   q3  | 50| 50|   0  |  R=0
% 4   q4  | 50| 50|  -90 |  R=0
% 5   q5  | 5 | 0 |   0  |  R=0

a1 = 0; alpha1 = 0; d1 = 50; theta1 = "q1";
a2 = 0; alpha2 = -pi/2; d2 = 0; theta2 = "q2";
a3 = 50; alpha3 = 0; d3 = 50; theta3 = "q3";
a4 = 50; alpha4 = -pi/2; d4 = 50; theta4 = "q4";
a5 = 0; alpha5 = 0; d5 = 5; theta5 = "q5";

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link1.Joint.setFixedTransform([a1 alpha1 d1 0],"dh");

link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform([a2 alpha2 d2 0],"dh");

link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([a3 alpha3 d3 0],"dh");

link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([a4 alpha4 d4 0],"dh");

link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([a5 alpha5 d5 0],"dh");

myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);

myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);

myRobot.showdetails
myRobot.show

%% Exercício 2

a = [a1, a2, a3, a4, a5];
alpha = [alpha1, alpha2, alpha3, alpha4, alpha5];
d = [d1, d2, d3, d4, d5];
theta = [0, 0, 0, 0, 0]; 


T = eye(4);


T_matrices = cell(1, numel(a));

for i = 1:numel(a)
    A = [
        cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
        sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
        0, sin(alpha(i)), cos(alpha(i)), d(i);
        0, 0, 0, 1;
    ];
    T = T * A;
    T_matrices{i} = T;
end


for i = 1:numel(T_matrices)
    fprintf('Matriz de transformação para a junta %d:\n', i);
    disp(T_matrices{i});
end

%% Exercício 3

T_matrices = cell(1, numel(a) + 1);
T_matrices{numel(a) + 1} = T;
for i = 1:numel(a)
    A = [
        cos(theta(i)), -sin(theta(i)) * cos(alpha(i)), sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
        sin(theta(i)), cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
        0, sin(alpha(i)), cos(alpha(i)), d(i);
        0, 0, 0, 1;
    ];
    T = T * A;
    T_matrices{i} = T;
end
disp('Matriz de transformação para o atuador:');
disp(T_matrices{end});

%% Exercício 4

clear all
close all

syms a1 a2 real
e = ETS3.Tz(a1)*ETS3.Rx("q1")*ETS3.Tx(a1)*ETS3.Tx(a1)*ETS3.Rz("q2")*ETS3.Tz(a2);

syms q1 q2 real
TE = e.fkine([q1 q2])
transl = TE(1:2,3)';
syms x y z real

e1 = x == transl(1)
e2 = y == transl(2)
e3 = z == transl(3)

[s1,s2] = solve([e1 e2],[q1 q2])

%% Exercício 5
% A(50,0,70), B(50,30,20) e C(50,-30,20) 
subs(s2(2),[a1 a2 a3], [50 0 70])
xfk = eval(subs(transl(1), [a1 a2 a3 q1 q2], [50 0 70 deg2rad(30) deg2rad(40)]))







