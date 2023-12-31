a1=1
a2=2

e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2);
TE = e.fkine([deg2rad(90) deg2rad(-90)])

%%
hold on
%Equações obtidas algebricamente

L1=1;
L2=1;

r = rateControl(10);
q = [linspace(0,2*pi,100)' linspace(0,2*pi,100)'];
for i = 1:size(q,1)
    x=sin(q(i,1));
    y=cos(q(i,2));
    theta2rad = acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2));
    theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)));
    
    e1 = ETS2.Rz("q1")*ETS2.Tx(L1);
    e2 = ETS2.Rz("q1")*ETS2.Tx(L1)*ETS2.Rz("q2")*ETS2.Tx(L2);
    t1=e1.fkine(theta1rad);
    t2=e2.fkine([theta1rad theta2rad]);
    
    %line([0 posX1 posX2],[0 posY1 posY2])
    line([0 t1(1,3) t2(1,3)],[0 t1(2,3) t2(2,3)])
    
    %myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
    r.waitfor;
end

%%
L1=1;
L2=1;
x=1.2080;
y=1.4397;
theta2rad = acos((x^2+y^2-L1^2-L2^2)/(2*L1*L2))
theta1rad = atan(y/x)-atan(L2*sin(theta2rad)/(L1+L2*cos(theta2rad)))
theta2 = rad2deg(theta2rad)
theta1 = rad2deg(theta1rad)

e1 = ETS2.Rz("q1")*ETS2.Tx(L1);
e2 = ETS2.Rz("q1")*ETS2.Tx(L1)*ETS2.Rz("q2")*ETS2.Tx(L2);
t1=e1.fkine(theta1rad);
t2=e2.fkine([theta1rad theta2rad]);

%line([0 posX1 posX2],[0 posY1 posY2])
line([0 t1(1,3) t2(1,3)],[0 t1(2,3) t2(2,3)])


%%
clear all
close all

syms a1 a2 real
e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2);

syms q1 q2 real
TE = e.fkine([q1 q2])
transl = TE(1:2,3)';

syms x y real
e1 = x == transl(1)
e2 = y == transl(2)

[s1,s2] = solve([e1 e2],[q1 q2])

length(s2)
%%
subs(s2(2),[a1 a2],[1 1])

xfk = eval(subs(transl(1), [a1 a2 q1 q2], [1 1 deg2rad(30) deg2rad(40)]))
yfk = eval(subs(transl(2), [a1 a2 q1 q2], [1 1 deg2rad(30) deg2rad(40)]))

q1r = eval(subs(s1(2),[a1 a2 x y],[1 1 xfk yfk]));
q1 = rad2deg(q1r)

q2r = eval(subs(s2(2),[a1 a2 x y],[1 1 xfk yfk]));
q2 = rad2deg(q2r)

%% Solução Numérica

e = ETS2.Rz("q1")*ETS2.Tx(1)*ETS2.Rz("q2")*ETS2.Tx(1);
pstar = [0.6 0.7]; % Desired position
q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0])
printtform2d(e.fkine(q),unit="deg")


pstar = [1.2080 1.4397];
q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),[0 0])
rad2deg(q)

%% Exercicio - Slide 15

e = ETS2.Rz("q1")*ETS2.Tx(50)*ETS2.Rz("q2")*ETS2.Tx(40)*ETS2.Rz("q3")*ETS2.Tx(30);

r = rateControl(10);
g = [linspace(0, 2*pi, 100)'];
for i = 1:size(g,1)
    pstar = [70+20*cos(g(i)), 70+20*sin(g(i))]; %posição desejada
    q = fminsearch(@(q) norm(se2(e.fkine(q)).trvec-pstar),q);
    %axis([0 100 0 100])
    e.plot(q, 'workspace', [0 100 0 100 0 1])
    
    r.waitfor;
end



%% Exercício Slide 17 - Cinemática Inversa do Robô Puma

e = ETS3.Tz(66)*ETS3.Rz("q1")*...
ETS3.Rx(pi/2)*ETS3.Rz("q2")*...
ETS3.Tx(43)*ETS3.Rz("q3")*...
ETS3.Tx(43)*ETS3.Rx("q4")*...
ETS3.Rz("q5")*ETS3.Rx("q6")*ETS3.Tx(5);

q = [0 0 0 0 0 0];
r = rateControl(10);
g = linspace(0, 2*pi, 100)';
for i = 1:size(g,1)
    pstar = [40 70+20*cos(g(i)) 70+20*sin(g(i)) 0 0 0]; %posição desejada
    q = fminsearch(@(q) norm([se3(e.fkine(q)).trvec se3(e.fkine(q)).eul]-pstar),q);
    rad2deg(q)
    %axis([0 100 0 100]
    e.plot(q, 'workspace', [-50 200 -50 200 -50 200], "scale", 0.1)
    
    r.waitfor;
end

%% Slide 22

abb = loadrobot("abbIrb1600",DataFormat="row");
abb.show;
% abb.showdetails
aIk = analyticalInverseKinematics(abb)
% aIk.showdetails

abb.getBody("tool0").Joint.Type
T = abb.getBody("tool0").Joint.JointToParentTransform;
printtform(T)

abbIkFcn = aIk.generateIKFunction("ikIRB1600")

gtPose = trvec2tform([0.93 0 0.5])*eul2tform([0 pi/2 0]);
qsol = abbIkFcn(gtPose)

% qn = qsol(1,:);
% abb.show(qn);

T1 = abb.getTransform(qsol(1,:),"tool0");
printtform(T1)
T2 = abb.getTransform(qsol(2,:),"tool0");
printtform(T2)

figure(1)
abb.show(qsol(1,:))
figure(2)
abb.show(qsol(2,:))

