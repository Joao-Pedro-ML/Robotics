a1 = 1
e = ETS2.Rz("q1")*ETS2.Tx(a1)
e.fkine(pi/6)

%e.teach

%%
a1 = 1
a2 = 1
e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2)
T = e.fkine(deg2rad([30 40]));
printtform2d(T,unit="deg")
e.teach

%%
a1 = 1
a2 = 1
%T = se2(deg2rad(30),"theta")*se2(eye(2),[a1 0]) * se2(deg2rad(40),"theta") 
e = ETS2.Rz("q1")*ETS2.Tx(a1)*ETS2.Rz("q2")*ETS2.Tx(a2)*ETS2.Rz("q3")*ETS2.Tx(a2)
e.njoints
e.structure
e.plot(deg2rad([30 40 20]));

%% 
e = ETS2.Rz("q1")*ETS2.Tx("q2",qlim=[1 2])
e.structure

%%
a1 = 1;
a2 = 2;
e = ETS3.Rz("q1")*ETS3.Ry("q2")* ...
    ETS3.Tz(a1)*ETS3.Ry("q3")*ETS3.Tz(a2)* ...
    ETS3.Rz("q4")*ETS3.Ry("q5")*ETS3.Rz("q6");
% e.njoints
% e.structure
% e.teach

%%
a1 = 1
a2 = 1
link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform(se3([a1 0 0],"trvec"));
link3 = rigidBody("link3");
link3.Joint.setFixedTransform(se3([a2 0 0],"trvec"));
myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.showdetails
T = myRobot.getTransform(deg2rad([30 40]),"link3");
printtform(T,unit="deg")
myRobot.show(deg2rad([30 40]));
view(0,90) % show only XY (2D) plane

q = [linspace(0,pi,100)' linspace(0,-2*pi,100)'];
whos q

r = rateControl(10);
for i = 1:size(q,1)
	myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
	r.waitfor;
end

link2 = myRobot.Bodies{2}
link2 = myRobot.getBody("link2");

parentLink = link2.Parent
childLinks = link2.Children
link2.Joint.Type
link2.Joint.PositionLimits
myRobot.getTransform(deg2rad([0 30]),"link2","link1")

%%
a1 = 1; a2 = 1;
robot6 = ets2rbt(ETS3.Rz("q1")*ETS3.Ry("q2")* ...
ETS3.Tz(a1)*ETS3.Ry("q3")*ETS3.Tz(a2)* ...
ETS3.Rz("q4")*ETS3.Ry("q5")*ETS3.Rz("q6"));
robot6.homeConfiguration
robot6.show(deg2rad([30 40 50 0 0 0]))

%% Exercício slide 16
l1 = 1; l2 = 2; l3 = 1; l4 = 0.5;
e = ETS3.Tz(l1)*ETS3.Rz("q1")*...
    ETS3.Ty(l2)*ETS3.Rx(-pi/2)*ETS3.Rz("q2")* ...
    ETS3.Tx(-l3)*ETS3.Tz(l4)*ETS3.Rz("q3")
e.teach

%%
panda = loadrobot("frankaEmikaPanda",DataFormat="row");
panda.showdetails
qr = [0 -0.3 0 -2.2 0 2 0.7854 0 0];
T = panda.getTransform(qr,"panda_hand");
printtform(T,unit="deg")
panda.show(qr);

%%
atlas = loadrobot("atlas",DataFormat="row");
atlas.show;
atlas.show([0 0 45 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]);

% childLinks = atlas.getBody("utorso").Children';
% size(childLinks)
% 
% cellfun(@(c) string(c.Name),childLinks)
% rightArm = atlas.subtree("r_clav");
% rightArm.show;




