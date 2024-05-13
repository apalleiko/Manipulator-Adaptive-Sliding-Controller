function [q,qd,qdd,pp] = create_cubic_traj(robot,weights,wpts,tpts,r,initial_pose)
%CREATE_TRAJ Create trajectory for a robot, given waypoints
ik = inverseKinematics('RigidBodyTree',robot);

% Get joint pos. of each pose
n = size(wpts,1); % Number of waypoints
solns = zeros(length(initial_pose),n);
last_pose = initial_pose;
for i=1:n
    tform = trvec2tform(wpts(i,:));
    [soln,~] = ik("arm3",tform,weights,last_pose);
    last_pose = soln;
    solns(:,i) = soln;
end

% Timing parameters
tf = tpts(end);
tvec = 0:r:tf;
% Just use a simple cubic polynomical trajectory solver
[q, qd, qdd, pp] = cubicpolytraj(solns, tpts, tvec);
end