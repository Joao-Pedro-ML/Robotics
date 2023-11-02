close all
clear all
%% Exercicio 1 

hold all
P = [1 2 3]'
plot3(1,2,3, '+', 'Color', 'k', 'MarkerSize', 10)
Rx = rotmx(pi/2)
iRx = inv(Rx)
Ry = rotmy(pi/2)
iRy = inv(Ry)
Rz = rotmz(pi/2)
iRz = inv(Rz)
Rotx = iRx*P
plot3(1,3,-2, "+", 'Color', 'g', 'MarkerSize', 10)
Roty = iRy*P
plot3(-3,2,1, "+", 'Color', 'r', 'MarkerSize', 10)
Rotz = iRz*P
plot3(2,-1,3, "+", 'Color', 'b', 'MarkerSize', 10)
grid

%% Exercicio 2

hold
Resp2 = rotmz(deg2rad(30))'*rotmy(deg2rad(20))'*[1 2 3]'


%% Exercicio 3

X = [0,0,0,0,0,0,0,0]';
Y = [-0.5,0.5,0.5,1,0,-1,-0.5,-0.5]';
Z = [0,0,0.5,0.5,1,0.5,0.5,0]';
ARROW = [X,Y,Z,0*X+1]'
T = eye(4,4);
Display3D(ARROW, T);

c = cos(5*pi/180);
s = sin(5*pi/180);
Rx = [1,0,0,0;0,c,s,0;0,-s,c,0;0,0,0,1];
T = eye(4,4);
for i=1:1000
    T = Rx*T;
    Display3D(ARROW, T);
    pause(0.01);
end

function Display3D(DATA, T)
    % scaling factor
    s = T(4,4);
    % draw the X, Y, Z axis
    X = [1,0,0,1]';
    Y = [0,1,0,1]';
    Z = [0,0,1,1]';
    O = [0,0,0,1]';
    DATA = T*DATA;
    T0 = T;
    T0(1,4) = 0;
    T0(2,4) = 0;
    T0(3,4) = 0;
    % transform
    X0 = T0*X;
    Y0 = T0*Y;
    Z0 = T0*Z;
    Origin = T0*O;
    % display is the y-z plane
    hold off;
    plot([-2,2],[-2,2],'wx');
    hold on;
    % Project onto the YZ axis
    Tx = s*[0,1,0,0];
    Ty = s*[0,0,1,0];
    plot(Tx*[Origin, X0], Ty*[Origin,X0], 'g');
    plot(Tx*[Origin, Y0], Ty*[Origin,Y0], 'r');
    plot(Tx*[Origin, Z0], Ty*[Origin,Z0], 'm');
    % display the data
    plot(Tx*DATA,Ty*DATA, 'b')
end




