function dxdt = sim_robot_robust_adaptive2(t,x,gamma,phi,lambda,w,bound,eta,q_traj,Y,Y_vars,q_ddot,vars)
% x is current state, u is function of time, x_est is estimated current
% state, if applicable.
t;
n = size(lambda,1); % Number of states

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
k = bound*ones(n,1) + eta;
s_d = s - phi*sat(s/phi);

qr_dot = qd_dot - lambda*q_tilda;
qr_ddot = qd_ddot - lambda*q_tilda_dot;

new_vars = [q;q_dot;qr_dot;qr_ddot];

Yt = double(subs(Y,Y_vars,new_vars));

wt = w(t);
tau = Yt*a_hat - k.*sat(s/phi) + wt;

a_hat_dot = -gamma*Yt'*s_d;

q_ddot = double(subs(q_ddot,vars,[q;q_dot;-tau]));
dxdt = [q_dot; q_ddot; a_hat_dot];
end

