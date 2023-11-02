cobra = loadrvcrobot("cobra");
TE = se3(deg2rad([180 0 30]),"eul","XYZ",[0.4 -0.3 0.2]);

cobraHome = cobra.homeConfiguration;
cobraIK = inverseKinematics(RigidBodyTree=cobra);
weights = ones(1,6);
rng(0); % obtain repeatable results
[qsol,solinfo] = cobraIK("link4",TE.tform,weights,cobraHome)

%%
weights = [0 0 1 1 1 1];
[qsol,solinfo] = cobraIK("link4",TE.tform,weights,cobraHome)

%%

T = cobra.getTransform(qsol, "link4")
printtform(T)
plotTransforms(TE,FrameColor="red"); hold on
plotTransforms(se3(T4),FrameColor="blue")

%%
panda = loadrobot("frankaEmikaPanda",DataFormat="row");

TE = se3(trvec2tform([0.7 0.2 0.1]))* ...
     se3(oa2tform([0 1 0],[0 0 -1]));
% configura-se a cinemática inversa do robo panda
pandaHome = panda.homeConfiguration;
pandaIK = inverseKinematics(RigidBodyTree=panda);
rng(0); % obtain repeatable results
% a solução da cinematica inversa é:
qsol = pandaIK("panda_hand",TE.tform,ones(1,6),pandaHome)
% a solução pode ser verificada:
handT = panda.getTransform(qsol,"panda_hand");
printtform(handT,unit="deg")
panda.show(qsol);


