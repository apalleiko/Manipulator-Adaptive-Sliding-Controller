function u = inverse_dynamics_controller(t,robot,q,qtd,qtdd,times)
% Compute torques required for a robot in a configuration at a given time
% in a trajectory
i = times <= t;

qti = q(:,i);
qt = qti(:,end);

qdti = qtd(:,i);
qdt = qdti(:,end);

qddti = qtdd(:,i);
qddt = qddti(:,end);

u = inverseDynamics(robot,qt,qdt,qddt);
end

