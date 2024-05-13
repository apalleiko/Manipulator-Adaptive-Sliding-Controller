function [tau, q_ddot, ddq, dq, q, u, vars] = robot_arm_dynamics()
    % Define joint states and inputs
    syms m1 m2 m3 L1 L2 l1 l2 t g theta1(t) theta2(t) tau1(t) tau2(t) 
    %m1:mass of link1
    %m2:mass of link2
    %L1:length of link1
    %L2:length of link2
    %l1:center of mass of link1
    %l2:center of mass of link2
    % Define symbolic variables with it
    vars = [m1 m2 m3 L1 L2 l1 l2 g];
    
    % Create forward dynamics
    dtheta1 = diff(theta1,t);
    dtheta2 = diff(theta2,t);
    I1 = (1/12)*m1*L1^2;
    I2 = (1/12)*m2*L2^2;

    % Define energies
    %KE of link1
    KE1=(1/2)*m1*(l1^2)*(dtheta1^2)+(1/2)*I1*dtheta1^2;
    %PE of link1
    P1=m1*g*l1*sin(theta1);
    %KE of link2
    KE2=(1/2)*m2*((L1^2)*(dtheta1^2)+(l2^2)*((dtheta1+dtheta2)^2)+2*L1*l2*cos(theta2)*(dtheta1*dtheta1+dtheta1*dtheta2))+(1/2)*I2*(dtheta1+dtheta2)^2;
    %PE of link2 
    P2=m2*g*(L1*sin(theta1)+l2*sin(theta1+theta2));
    %KE of EE
    KE3=(1/2)*m3*((L1^2)*(dtheta1^2)+(L2^2)*((dtheta1+dtheta2)^2)+2*L1*L2*cos(theta2)*(dtheta1*dtheta1+dtheta1*dtheta2));
    %PE of EE
    P3=m3*g*(L1*sin(theta1)+L2*sin(theta1+theta2));

    Le = KE1+KE2+KE3-P1-P2-P3;
   
    q = [theta1; theta2];
    u = [tau1; tau2];
    dq = diff(q,t);
    
    L_mat = Le;
    dLdq = jacobian(L_mat,q);
    dLdqd = jacobian(L_mat,dq);
    EL = dLdq-diff(dLdqd,t);
    EL_eq = transpose(EL) == [tau1;tau2];
    EL_eq_simp = formula(simplify(EL_eq));
    
    % Solve expression for q_ddot
    ddq = [diff(theta1(t), t, t), diff(theta2(t), t, t)];
    syms d2q_1 d2q_2
    EL_eq_sym = subs(EL_eq_simp, ddq,[d2q_1, d2q_2]);
    q_ddot = solve(EL_eq_sym,[d2q_1, d2q_2]);
    q_ddot = [q_ddot.d2q_1;q_ddot.d2q_2];

    % Also return tau form
    tau = formula(transpose(EL));
end