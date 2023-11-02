%% Exercicio 3

X = [0,0,0,0,0,0,0,0]';
Y = [-0.5,0.5,0.5,1,0,-1,-0.5,-0.5]';
Z = [0,0,0.5,0.5,1,0.5,0.5,0]';
ARROW = [X,Y,Z,0*X+1]'

% Rotacionar a seta em relação a X com Rx e a Y com Ry

Ry = [c,0,-s,0;...
      0,1,0,0;...
      s,0,c,0;...
      0,0,0,1];

c = cos(-45*pi/180);
s = sin(-45*pi/180);

Rz = [c,s,0,0;...
      -s,c,0,0;...
      0,0,1,0;...
      0,0,0,1];

c = cos(-45*pi/180);
s = sin(-45*pi/180);

grid
Tdist = Ry*Rz
%MOVE O OBJETO
for i=1:300
    T(3,4) = i/100;
    Display3D(T*ARROW, Tdist);
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

