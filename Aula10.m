t = linspace(0,1,50); % 0 to 1 in 50 steps
[q,qd,qdd] = quinticpolytraj([0 1],[0 1],t);
stackedplot(t,[q' qd' qdd'])

%%
[q2,qd2,qdd2] = quinticpolytraj([0 1],[0 1],t, ...
                VelocityBoundaryCondition=[10 0]);
                stackedplot(t,[q2' qd2' qdd2'])
%%
[q,qd,qdd] = trapveltraj([0 1],50);
stackedplot(t,[q' qd' qdd'])

%%
[q2,qd2,qdd2] = trapveltraj([0 1],50,EndTime=1, PeakVelocity=1.2);
[q3,qd3,qdd3] = trapveltraj([0 1], 50, EndTime=1, PeakVelocity=2);
figure(1)
stackedplot(t,[q2' qd2' qdd2'])
figure(2)
stackedplot(t,[q3' qd3' qdd3'])

%%
q0 = [0 2]; qf = [1 -1];
q = trapveltraj([q0' qf'],50,EndTime=1);


%%

cornerPoints = [-1 1; 1 1; 1 -1; -1 -1; -1 1];
R = so2(deg2rad(30),"theta");
via = R.transform(cornerPoints)';
[q,qd,qdd,t] = trapveltraj(via,100,EndTime=5);
q2 = trapveltraj(via,100,EndTime=5,AccelTime=2);

plot(q(1,:), q(2,:), ".-")

%%
[q,qd,qdd] = minjerkpolytraj(via,[1 2 3 4 5],500);

figure(1)
subplot(1,2,1)
plot(q(1,:),q(2,:),".-")

t = linspace(0,1,500);
subplot(1,2,2)
plot(t, q')

%% Slide 14

T0 = se3(eul2rotm([1.5 0 0]),[0.4 0.2 0]);
T1 = se3(eul2rotm([-pi/2 0 -pi/2]),[-0.4 -0.2 0.3]);
%Cria se uma trajetória linear entre com 50 pontos
tpts = [0 1]; tvec = linspace(tpts(1),tpts(2),50);
Ts = transformtraj(T0,T1,tpts,tvec);
%A trajetória pode ser visualizada com o comando:
animtform(Ts)

%%
Ts = transformtraj(T0,T1,tpts,trapveltraj(tpts,50));










