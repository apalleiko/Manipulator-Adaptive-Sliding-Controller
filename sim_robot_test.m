function [t,x] = sim_robot_test(qddot,vars,u,x0,tspan)
% simulate the nonlinear response of a robot to given inputs
[t,x] = ode45(@(t,x) robot_dynamics(t,x,u,qddot,vars),tspan,x0);
end

function dxdt = robot_dynamics(t,x,u,qddot,vars)
% x is current state, u is function of time
n = length(x)/2; % number of states
q = x(1:n); % get current configuration of the robot
q_dot = x(n+1:end); % get current velocities of the robot

% Assume F_ext is zero
tau = double(u(t));
q_ddot = double(subs(qddot,formula(vars),[q;q_dot;tau]));

dxdt = [q_dot; q_ddot];
end