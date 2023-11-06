%% Exercício 1
a1 = 0; alpha1 = 0; d1 = 50; theta1 = "q1";
a2 = 50; alpha2 = -pi/2; d2 = 0; theta2 = "q2";
a3 = 50; alpha3 = 0; d3 = 0; theta3 = "q3";
a4 = 0; alpha4 = -pi/2; d4 = 0; theta4 = "q4";
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

syms q1 q2 q3 q4 q5; 

dh_parameters = [
    0, 0, 50, q1;
    50, -pi/2, 0, q2;
    50, 0, 0, q3;
    0, -pi/2, 0, q4;
    0, 0, 5, q5;
];

T = eye(4);

for i = 1:size(dh_parameters, 1)
    a = dh_parameters(i, 1);
    alpha = dh_parameters(i, 2);
    d = dh_parameters(i, 3);
    theta = dh_parameters(i, 4);
    
    A = [
        cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
        0, sin(alpha), cos(alpha), d;
        0, 0, 0, 1
    ];
    
    T = T * A;
    
    fprintf('Matriz de transformação homogênea da Junta %d:\n', i);
    disp(A);
    fprintf('\n');
end

%% Exercicio 3

fprintf('Matriz de transformação homogênea do atuador em função dos ângulos das juntas:\n');
disp(T);

%% Exercicio 4 e 5
% A(50,0,70), B(50,30,20) e C(50,-30,20)
%e = ETS2.Tx(50) * ETS2.Rz("q1") * ETS2.Tx(100) * ETS2.Rz("q2") * ETS2.Tx(5);
e = ETS3.Tz(50)*ETS3.Rx("q1")*ETS3.Tx(50)*ETS3.Tx(50)*ETS3.Rz("q2")*ETS3.Tz(5);

pstar = [50, 0, 70]; %posição desejada
q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0 0])
e.plot(q, 'workspace')

%%








