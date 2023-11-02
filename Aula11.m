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

