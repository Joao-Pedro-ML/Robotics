%% Trabalho 2 - Robótica
% Exercício 1

a1 = 2.075; alpha1 = 0; d1 = 0; theta1 = "q1";
a2 = 0; alpha2 = -pi/2; d2 = 3.375; theta2 = "q2";
a3 = 0; alpha3 = 0; d3 = -2.85; theta3 = "q3";
a4 = 0; alpha4 = 0; d4 = -8.84; theta4 = "q4";
a5 = 0; alpha5 = 0; d5 = -6.84; theta5 = "q5";

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
myRobot.addBody(link2, link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5, link4.Name);

myRobot.show

syms q1 q2 q3 q4 q5; 

dh_parameters = [
    2.075, 0, 0, q1;
    0, -pi/2, 3.375, q2;
    0, 0, -2.85, q3;
    0, 0, -8.84, q4;
    0, 0, -6.84, q5;
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

%% Exercícios 2, 3, 4 e 5
%q1 em x
%q2 e q2 em y

L1 = 0.5;
L2 = 0.5;
e = ETS3.Rx("q1")*ETS3.Ty(-0.2075)*ETS3.Tx(0.3375)*...
    ETS3.Tz(-0.2850)*ETS3.Ry("q2")*ETS3.Tz(-0.8840)*ETS3.Ry("q3")*ETS3.Tz(-0.6840);
e2 = ETS3.Rx("q1")*ETS3.Ty(0.2075)*ETS3.Tx(0.3375)*...
    ETS3.Tz(-0.2850)*ETS3.Ry("q2")*ETS3.Tz(-0.8840)*ETS3.Ry("q3")*ETS3.Tz(-0.6840);
%Modifica de ETS para o sistema de corpo rígido de robôs
leg = ets2rbt(e);
leg2 = ets2rbt(e2);
figure(1)
%se3(leg.getTransform([0 0 0],"link8")).trvec
leg.show([deg2rad(-10) deg2rad(45) deg2rad(-45)]);
figure(2)
leg2.show([deg2rad(-10) deg2rad(45) deg2rad(-45)]);

%% qcycle frente-esquerda - trajetória da perna para frente
xf = (40+33.75)/100; xb = (-40+33.75)/100; y = -20.75/100; zu = -120/100; zd = -140/100;
via = [xf y zd
xb y zd
xb y zu
xf y zu
xf y zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
global qcycle;
qcycle = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle2 frente-direita - trajetória da perna para frente
xf = (-40+33.75)/100; xb = (40+33.75)/100; y = 20.75/100; zu = -120/100; zd = -140/100;
via = [xf y zd
xb y zd
xb y zu
xf y zu
xf y zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
global qcycle2;
qcycle2 = ikineTrajNum(leg2,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle3 para o lado frente - trajetória da perna para o lado
xf = (33.75)/100; xb = (33.75)/100; yf = (30-20.75)/100; y = -30/100-20.75/100; zu = -120/100; zd = -140/100;
via = [xf yf zd
xb y zd
xb y zu
xf yf zu
xf yf zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
global qcycle3;
qcycle3 = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle4 para o lado tras - trajetória da perna para o lado
xf = (33.75)/100; xb = (33.75)/100; yf = (-30-20.75)/100; y = 30/100-20.75/100; zu = -120/100; zd = -140/100;
via = [xf yf zd
xb y zd
xb y zu
xf yf zu
xf yf zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
global qcycle4;
qcycle4 = ikineTrajNum(leg2,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle5 para o circulo - trajetória da perna 
raio=150
xi = (raio*cos(deg2rad(10))-100)/100; xf = (raio*cos(deg2rad(55))-100)/100; ...
    yi = -(raio*sin(deg2rad(10))-50)/100; yf = -(raio*sin(deg2rad(55))-50)/100;...
    zu = -120/100; zd = -140/100;
via = [xi yi zd
xf yf zd %pe baixo
xf yf zu %pe alto
xi yi zu %pe alto
xi yi zd]; %pe baixo
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
global qcycle5;
qcycle5 = ikineTrajNum(leg2,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);
%% qcycle6 para o circulo - trajetória da perna 
%raio do robo raiz(100^2+50^2) = 112
%y vai de yi -30,56 a yf 41,74 -> 112*sen(10)-50 e 112*sen(55)-50
%x 112*cos(10 graus) - 100 e 112*cos(55 graus) - 100 -> xi 10,298 e xf -35,75
%x vai de 10,2 a -28
xf = (raio*cos(deg2rad(10))-100)/100; xi = (raio*cos(deg2rad(55))-100)/100; ...
    yf = -(raio*sin(deg2rad(10))-50)/100; yi = -(raio*sin(deg2rad(55))-50)/100;...
    zu = -120/100; zd = -140/100;
via = [xi yi zd
xf yf zd %pe baixo
xf yf zu %pe alto
xi yi zu %pe alto
xi yi zd]; %pe baixo
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
global qcycle6;
qcycle6 = ikineTrajNum(leg,se3(eye(3),x),"link8", ...
weights=[0 0 0 1 1 1]);

%%
global r;
r = rateControl(200);
while(1)
    for i = 1:size(qcycle5,1)
        leg2.show(qcycle5(i,:),FastUpdate=true,PreservePlot=false);
        r.waitfor;
    end
end

%% animação do robô com as quatro pernas realizando os movimentos para frente
close all
W = 100/100; L = 200/100;
Tflip = se3(pi,"rotz");
global legs;
legs = [ ...
rbtTform(leg,se3(eye(3),[L/2 -W/2 0])), ...         %frente
rbtTform(leg,se3(eye(3),[-L/2 W/2 0])*Tflip), ...   %atras
rbtTform(leg2,se3(eye(3),[-L/2 -W/2 0])*Tflip), ... %frente
rbtTform(leg2,se3(eye(3),[L/2 W/2 0]))];            %atras
r = rateControl(200);

%% Para girar, frente e lado
while(1)
    girar(400);
    frente(400);
    ladoes(400);
    ladodir(400);
end
%% para frente
function [] = frente(qtd)
global legs qcycle2 qcycle r
    for i = 1:qtd
    i=i+1;
 	legs(1).show(gait(qcycle2,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	legs(2).show(gait(qcycle,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle2,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
    end
end

%% Para o lado esquerda
function [] = ladoes(qtd)
global legs r qcycle4 qcycle3
    for i = 1:qtd
    i=i+1;
 	legs(1).show(gait(qcycle4,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	 legs(2).show(gait(qcycle3,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle4,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle3,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
    end
end

%% Para o lado direita
function [] = ladodir(qtd)
global legs r qcycle4 qcycle3
    for i = 1:qtd
    i=i+1;
 	legs(1).show(gait(qcycle3,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	legs(2).show(gait(qcycle4,i,100,false),FastUpdate=true,PreservePlot=false);
 	legs(3).show(gait(qcycle3,i,200,true),FastUpdate=true,PreservePlot=false);
 	legs(4).show(gait(qcycle4,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
    r.waitfor;
    end
end
%% Para girar
function [] = girar(qtd)
global legs r qcycle5 qcycle6
    for i = 1:qtd
        i=i+1;
 	    legs(1).show(gait(qcycle6,i,0,false),FastUpdate=true,PreservePlot=false); hold on;
	    legs(2).show(gait(qcycle6,i,100,false),FastUpdate=true,PreservePlot=false);
 	    legs(3).show(gait(qcycle5,i,200,true),FastUpdate=true,PreservePlot=false);
 	    legs(4).show(gait(qcycle5,i,300,true),FastUpdate=true,PreservePlot=false); hold off;
        r.waitfor;
    end
end
%% Para girar, frente e lado








