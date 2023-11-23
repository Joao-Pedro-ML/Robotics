%Carregando as bibliotecas do robô ABB1600
abb = loadrobot("abbIrb1600",DataFormat="row");
aIK = analyticalInverseKinematics(abb);
abbIKFcn = aIK.generateIKFunction("ikIRB1600")

%Considerando a posição inicial e final como:
TE1 = se3(3,"rotx",[0.6 -0.5 0.1]);
TE2 = se3(2,"rotx",[0.4 0.5 0.1]);

%%
%Encontrando as soluções da cinemática inversa para as posições inicial e final temos:
sol1 = abbIKFcn(TE1.tform);
sol2 = abbIKFcn(TE2.tform);
waypts = [sol1(1,:)' sol2(1,:)'];

%%
%Cria-se um vetor de tempo
t = 0:0.02:2;

%Utiliza-se o polinômio de quinto grau ou velocidade trapezoidal
[q,qd,qdd] = quinticpolytraj(waypts,[0 2],t);
[q,qd,qdd] = trapveltraj(waypts,numel(t),EndTime=2);

%%
%Para a visualização do movimento do robô
r = rateControl(50);
for i = 1:size(q,2)
    abb.show(q(:,i)',FastUpdate=true,PreservePlot=false);
    r.waitfor;
end

%%
for i = 1:size(q,2)
    trajT(i) = se3(abb.getTransform(q(:,i)',"tool0"));
end
size(trajT)

p = trajT.trvec;
size(p)

plot(t,trajT.eul("XYZ"))
legend("roll","pitch","yaw")
%%
%Podemos verificar os valores de cada junta graficamente
xplot(t,q')

%% Exercicio Slide 9 - Parte 1

TE1 = se3(3,"rotx",[1, -0.5, 0.7]);
TE2 = se3(3,"rotx",[1, 0.5, 0.7]);
TE3 = se3(2, "rotx", [1, -0.5, 0.2]);
TE4 = se3(3, "rotx", [1, 0.5, 0.2]);
sol1 = abbIKFcn(TE1.tform)
sol2 = abbIKFcn(TE2.tform)
sol3 = abbIKFcn(TE3.tform)
sol4 = abbIKFcn(TE4.tform)
waypts = [sol1(1,:)' sol2(1,:)' sol3(1,:)' sol4(1,:)'];

[q,qd,qdd] = trapveltraj(waypts,numel(t),EndTime=2);

r = rateControl(50);
while(1)
    for i = 1:size(q,2)
        abb.show(q(:,i)',FastUpdate=true,PreservePlot=false);
        r.waitfor;
    end
end

%% Exercicio - Parte 2
cornerPoints = [1,-0.5,0.7; 1,0.5,0.7;1,-0.5,0.2;1,0.5,0.2];
R = so3(deg2rad(30),"roty");
via = R.transform(cornerPoints);
A = via(1,:);
B = via(2,:);
C = via(3,:);
D = via(4,:);
solA = abbIKFcn(TE1.tform)
solB = abbIKFcn(TE2.tform)
solC = abbIKFcn(TE3.tform)
solD = abbIKFcn(TE4.tform)
waypts = [solA(1,:)' solB(1,:)' solC(1,:)' solD(1,:)' solA(1,:)'];

[q,qd,qdd] = trapveltraj(waypts,numel(t),EndTime=2);

r = rateControl(50);
while(1)
    for i = 1:size(q,2)
        abb.show(q(:,i)',FastUpdate=true,PreservePlot=false);
        r.waitfor;
    end
end

%% Parte 3

for i = 1:size(q,2)
    trajT(i) = se3(abb.getTransform(q(:,i)',"tool0"));
end
p = trajT.trvec;
plot3(q(1,:), q(2,:), q(3,:), ".-")
legend("roll","pitch","yaw")
%%

plot(t, p)
legend("x", "y", "z")
%%
plot(t, trajT.eul("XYZ"))
legend("rol1", "pitch", "yaw")

%% Slide 10

T = transformtraj(TE1,TE2,[0 2],t);
[s,sd,sdd] = minjerkpolytraj([0 1],[0 2],numel(t));
Ts = transformtraj(TE1,TE2,[0 2],t,TimeScaling=[s;sd;sdd]);

%Pode se visualizar os valores de translação e rotação do efetuador:
figure(1)
plot(t,Ts.trvec);
legend("x","y","z")
figure(2)
plot(t,Ts.eul("XYZ"));
legend("roll","pitch","yaw")
%Pode ser utilizar a cinemática inversa com o comando
qj = ikineTraj(abbIKFcn,Ts);
%Animação
figure(3)
r = rateControl(10);
for i = 1:size(qj,1)
	abb.show(qj(i,:),FastUpdate=true,PreservePlot=false);
	r.waitfor;
end

%% Slide 15

%Carrega os dados do robô ABB
abb = loadrobot("abbIrb1600",DataFormat="row");
aIK = analyticalInverseKinematics(abb);
abbIKFcn = aIK.generateIKFunction("ikIRB1600")
%Carrega a biblioteca de caracteres
load hershey
B = hershey{'B'}
%Prepara a trajetória a ser percorrida pelo efetuador
p = [0.5*B.stroke; zeros(1,size(B.stroke,2))];
k = find(isnan(p(1,:)));
p(:,k) = p(:,k-1); p(3,k) = 0.2;
dt = 0.02; % sample interval
traj = mstraj(p(:,2:end)',[0.5 0.5 0.5],[],p(:,1)',dt,0.2);
whos traj
%% Slide 18
%Trajetória 
fprintf("Tamanho da trajetória: %i\n", size(traj,1)*dt)

plot3(traj(:,1),traj(:,2),traj(:,3))
%%
%Cria a trajetória com as matrizes de transformação
Tp = se3(trvec2tform([0.5 0 0.075]))*se3(eye(3),traj)*se3(oa2tform([0 1 0],[0 0 -1]));
%Encontra os angulos de cada junta para cada ponto da trajetória utilizando a cinemática inversa
qj = ikineTraj(abbIKFcn,Tp);
%Animação
r = rateControl(1/dt);
for i = 1:size(qj,1)
    abb.show(qj(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end

%% Slide 20

%Define o tamanho dos segmentos da perna e as matrizes de transformação
L1 = 0.5;
L2 = 0.5;
e = ETS3.Rz("q1")*ETS3.Rx("q2")*ETS3.Ty(-L1)*ETS3.Rx("q3")*ETS3.Tz(-L2);

%Modifica de ETS para o sistema de corpo rígido de robôs
leg = ets2rbt(e);
se3(leg.getTransform([0 0 0],"link5")).trvec
leg.show;

%%
xf = 0.25; xb = -xf; y = -0.25; zu = -0.1; zd = -0.25;
via = [xf y zd ...
       xb y zd ...
       xb y zu ...
       xf y zu ...
       xf y zd];
x = mstraj(via,[],[3 0.25 0.5 0.25],[],0.01,0.1);
qcycle = ikineTrajNum(leg,se3(eye(3),x),"link5", weights=[0 0 0 1 1 1]);
r = rateControl(50);
for i = 1:size(qcycle,1)
    leg.show(qcycle(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end

%%
W = 0.5; L = 1;
Tflip = se3(pi,"rotz");
legs = [rbtTform(leg,se3(eye(3),[L/2 W/2 0])*Tflip), ...
        rbtTform(leg,se3(eye(3),[-L/2 W/2 0])*Tflip), ...
        rbtTform(leg,se3(eye(3),[L/2 -W/2 0])), ...
        rbtTform(leg,se3(eye(3),[-L/2 -W/2 0]))];
r = rateControl(50);
for i = 1:500
 	legs(1).show(gait(qcycle,i,0,false)); hold on;
	 legs(2).show(gait(qcycle,i,100,false));
 	legs(3).show(gait(qcycle,i,200,true));
 	legs(4).show(gait(qcycle,i,300,true)); hold off;
 	r.waitfor;
end

function q = gait(cycle, k, phi, flip)
    k = mod(k+phi-1,size(cycle,1))+1;
    q = cycle(k,:);
    if flip
        q(1) = -q(1); % for left-side legs
    end
end




