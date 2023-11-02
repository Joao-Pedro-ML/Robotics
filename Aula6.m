%% Robo Puma Slide 6

%    teta | d | a | alfa | sigma
% 1   q1  |660| 0 |  -90 |  R=0
% 2   q2  | 50|432|   0  |  R=0
% 3   q3  | 0 | 0 |  90  |  R=0
% 4   q4  |432| 0 |  -90 |  R=0
% 5   q5  | 0 | 0 |  90  |  R=0
% 6   q6  | 0 | 0 |   0  |  R=0

a1 = 0; alpha1 = -pi/2; d1 = 660; theta1 = "q1";
a2 = 432; alpha2 = 0; d2 = 50; theta2 = "q2";
a3 = 0; alpha3 = pi/2; d3 = 0; theta3 = "q3";
a4 = 0; alpha4 = -pi/2; d4 = 432; theta4 = "q4";
a5 = 0; alpha5 = pi/2; d5 = 0; theta5 = "q5";

link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","revolute");
link1.Joint.setFixedTransform([0 -pi/2 660 0],"dh");

link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","revolute");
link2.Joint.setFixedTransform([432 0 50 0],"dh");

link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([0 pi/2 0 0],"dh");

link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([0 -pi/2 432 0],"dh");

link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([0 pi/2 0 0],"dh");

link6 = rigidBody("link6");
link6.Joint = rigidBodyJoint("joint6","revolute");
link6.Joint.setFixedTransform([0 0 0 0],"dh");


myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);
myRobot.addBody(link6,link5.Name);

myRobot.showdetails

q = [linspace(0,pi,100)' linspace(0,-2*pi,100)' linspace(0,-2*pi,100)' ...
     linspace(0,-2*pi,100)' linspace(0,-2*pi,100)' linspace(0,-2*pi,100)'];
whos q

r = rateControl(10);
for i = 1:size(q,1)
	myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
	r.waitfor;
end

myRobot.show

%% Slide 12

%     teta | d | a | alfa
%  1    0  | 10| 0 |  0
%  2    q1 | 0 | 0 |  90     
%  3    q2 | 0 | 50|  0    
%  4    q3 | 0 | 40|  0    
%  5    0  | 0 | 30|  0   


a1 = 0; alpha1 = 0; d1 = 10; theta1 = 0;
a2 = 0; alpha2 = pi/2; d2 = 0; theta2 = "q1";
a3 = 50; alpha3 = 0; d3 = 0; theta3 = "q2";
a4 = 40; alpha4 = 0; d4 = 0; theta4 = "q3";
a5 = 30; alpha5 = 0; d5 = 0; theta5 = 0;


link1 = rigidBody("link1");
link1.Joint = rigidBodyJoint("joint1","fixed");
%                            a, alpha, d, theta  
link1.Joint.setFixedTransform([0 0 10 0],"dh");

link2 = rigidBody("link2");
link2.Joint = rigidBodyJoint("joint2","fixed");
link2.Joint.setFixedTransform([0 pi/2 0 0], "dh");

link3 = rigidBody("link3");
link3.Joint = rigidBodyJoint("joint3","revolute");
link3.Joint.setFixedTransform([50 0 0 0], "dh");

link4 = rigidBody("link4");
link4.Joint = rigidBodyJoint("joint4","revolute");
link4.Joint.setFixedTransform([40 0 0 0], "dh");

link5 = rigidBody("link5");
link5.Joint = rigidBodyJoint("joint5","revolute");
link5.Joint.setFixedTransform([30 0 0 0], "dh");

myRobot = rigidBodyTree(DataFormat="row");
myRobot.addBody(link1,myRobot.BaseName);
myRobot.addBody(link2,link1.Name);
myRobot.addBody(link3,link2.Name);
myRobot.addBody(link4,link3.Name);
myRobot.addBody(link5,link4.Name);

myRobot.showdetails

q = [linspace(0,pi/3,100)' linspace(0, -pi/3, 100)' linspace(0,-pi/3,100)'];
whos q

r = rateControl(10);
for i = 1:size(q,1)
	myRobot.show(q(i,:),FastUpdate=true,PreservePlot=false);
	r.waitfor;
end

myRobot.show







