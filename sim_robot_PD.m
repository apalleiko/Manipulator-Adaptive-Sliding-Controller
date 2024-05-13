function [t,x] = sim_robot_PD(robot,Kp,Kd,q_traj,x0,tspan)
% simulate the nonlinear response of a robot to given inputs
% Assume we know the true state for control
    [t,x] = ode45(@(t,x) robot_dynamics(t,x,Kp,Kd,q_traj,robot),tspan,x0);
end

function dxdt = robot_dynamics(t,x,Kp,Kd,q_traj,robot)
% x is current state, u is function of time, x_est is estimated current
% state, if applicable.

n = length(x)/2; % number of states
q = x(1:n); % get current configuration of the robot
q_dot = x(n+1:end); % get current velocities of the robot

% Get time changing vars
qd_t = q_traj(t);

dq = q - qd_t(1:n);
dqd = q_dot - qd_t(n+1:end);
tau = -Kp*dq - Kd*dqd;
q_ddot = forwardDynamics(robot,q,q_dot,tau,[]);

dxdt = [q_dot; q_ddot];
end