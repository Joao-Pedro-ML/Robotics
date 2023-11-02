clear all
close all
TO=trvec2tform([0 0])
plottform2d(TO,frame="O",color="k")
TX=trvec2tform([2 3])
plottform2d(TX,frame="X",color="r")
axis([-5 7 -1 5])
grid
TR=tformr2d(2)
plottform2d(TR,frame="R",color="b")
plottform2d(TX*TR,frame="XR",color="g")
plottform2d(TR*TX,frame="RX",color="y")
viscircles([0 0],sqrt(13))
C=[3 2]
plotpoint(C,"ko", label="C")
TC=trvec2tform(C)*TR*trvec2tform(-C)*TX
plottform2d(TC,frame="TC",color="m")
viscircles(C,sqrt(2))
%%
close all
alpha=30
beta=90
ca=cos(deg2rad(alpha))
sa=sin(deg2rad(alpha))
cb=cos(deg2rad(beta))
sb=sin(deg2rad(beta))

RA=[ca -sa 0;...
sa ca 0;...
0 0 1]

Tl1=[1 0 0.6;...
0 1 0;...
0 0 1]

RB=[cb -sb 0;...
sb cb 0;...
0 0 1]

Tl2=[1 0 0.7;...
0 1 0;...
0 0 1]

rl1=RA*Tl1
rl2=RA*Tl1*RB*Tl2

line([0 rl1(1,3) rl2(1,3)],[0 rl1(2,3) rl2(2,3)])
axis([0 1.5 0 1.5])

%%
close all

alpha=-30
beta=90

TR1=tformr2d(deg2rad(alpha))*trvec2tform([0.6 0])

TR2=TR1*tformr2d(deg2rad(beta))*trvec2tform([0.7 0])

line([0 TR1(1,3) TR2(1,3)],[0 TR1(2,3) TR2(2,3)])
axis([-1.5 1.5 -1.5 1.5])
grid

%%