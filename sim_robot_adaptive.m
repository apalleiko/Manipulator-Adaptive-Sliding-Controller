function dxdt = sim_robot_adaptive(t,x,gamma,Kd,lambda,q_traj,Y,Y_vars,robot)
% x is current state, u is function of time, x_est is estimated current
% state, if applicable.

n = size(Kd,1); % Number of states

q = x(1:n);
q_dot = x(n+1:2*n);
a_hat = x(2*n+1:end);

q_traj_t = q_traj(t);
qd = q_traj_t(1:n);
qd_dot = q_traj_t(n+1:2*n);
qd_ddot = q_traj_t(2*n+1:end);

q_tilda = q-qd;
q_tilda_dot = q_dot-qd_dot;

s = q_tilda_dot + lambda*q_tilda;

qr_dot = qd_dot - lambda*q_tilda;
qr_ddot = qd_ddot - lambda*q_tilda_dot;

new_vars = [q;q_dot;qr_dot;qr_ddot];

Yt = double(subs(Y,Y_vars,new_vars));

tau = Yt*a_hat - Kd*s;

a_hat_dot = -gamma*Yt'*s;

q_ddot = forwardDynamics(robot,q,q_dot,tau,[]);
dxdt = [q_dot; q_ddot; a_hat_dot];
end

