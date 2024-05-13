function [tau, q_ddot, ddq, dq, q, u, vars] = robot_arm_dynamics2()
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

    cm1 = [l1*cos(theta1);
           l1*sin(theta1)];
    cm2 = [L1*cos(theta1)+l2*cos(theta1+theta2);
           L1*sin(theta1)+l2*sin(theta1+theta2)];
    cm3 = [L1*cos(theta1)+L2*cos(theta1+theta2);
           L1*sin(theta1)+L2*sin(theta1+theta2);];

    % Define energies
    %KE of link1
    KE1=(1/2)*m1*sum(diff(cm1,t).*diff(cm1,t))+(1/2)*I1*(dtheta1)^2;
    KE1=simplify(KE1);
    %PE of link1
    P1=m1*g*l1*sin(theta1);
    %KE of link2
    KE2=(1/2)*m2*sum(diff(cm2,t).*diff(cm2,t))+(1/2)*I2*(dtheta1+dtheta2)^2;
    KE2=simplify(KE2);
    %PE of link2 
    P2=m2*g*(L1*sin(theta1)+l2*sin(theta1+theta2));
    %KE of EE
    KE3=(1/2)*m3*sum(diff(cm3,t).*diff(cm3,t));
    KE3=simplify(KE3);
    %PE of EE
    P3=m3*g*(L1*sin(theta1)+L2*sin(theta1+theta2));

    Le = KE1+KE2+KE3-P1-P2-P3;
   
    q = [theta1; theta2];
    u = [tau1; tau2];
    dq = diff(q,t);
    
    L_mat = Le;
    dLdq = formula(jacobian(L_mat,q));
    dLdqd = formula(jacobian(L_mat,dq));
    EL = formula(dLdq-diff(dLdqd,t));
    EL_eq = transpose(EL) == [tau1; tau2];
    EL_eq_simp = simplify(EL_eq);
    
    % Solve expression for q_ddot
    ddq = [diff(theta1(t), t, t), diff(theta2(t), t, t)];
    syms d2q_1 d2q_2
    EL_eq_sym = subs(EL_eq_simp, ddq,[d2q_1, d2q_2]);
    q_ddot = solve(EL_eq_sym,[d2q_1, d2q_2]);
    q_ddot = [q_ddot.d2q_1;q_ddot.d2q_2];

    % Also return tau form
    tau = formula(transpose(EL));
end