function [robot] = create_robot_tree(L)
% Create associated robot tree as robot_arm_dynamics.m to visualize things 
% L is the length of each arm
robot = rigidBodyTree("DataFormat","column");
base = robot.Base;
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
arm3 = rigidBody("arm3");

jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jnt3 = rigidBodyJoint("jnt3","fixed");

jnt1.JointAxis = [1 0 0]; % y-axis
% jnt1.HomePosition = pi/4;
jnt1.HomePosition = 0;
jnt2.JointAxis = [1 0 0];
% jnt2.HomePosition = pi/4;
jnt2.HomePosition = 0;

j_vec = [0 L 0];

setFixedTransform(jnt1,trvec2tform([0 0 0]))
setFixedTransform(jnt2,trvec2tform(j_vec))
setFixedTransform(jnt3,trvec2tform(j_vec))

arm1.Joint = jnt1;
arm1.CenterOfMass = j_vec/2;
arm2.Joint = jnt2;
arm2.CenterOfMass = j_vec/2;
arm3.Joint = jnt3;

t_mat = tform(se3([0,0,pi/2],'eul',j_vec/2));

addVisual(arm1,"Cylinder",[.05, L],t_mat);
addVisual(arm1,"Sphere",.075);
addVisual(arm2,"Cylinder",[.05, L],t_mat);
addVisual(arm2,"Sphere",.075)
addVisual(arm3,"Sphere",.1);

addBody(robot,arm1,"base");
addBody(robot,arm2,"arm1");
addBody(robot,arm3,"arm2");

% showdetails(robot)
% show(robot)
% xlim([-.01,.1])
% view(90,0)
end

