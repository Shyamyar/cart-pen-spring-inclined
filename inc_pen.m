function sdot = inc_pen(t,s,K,L,M,m,beta)
g = 9.81; %gravitational acceleration (m/s^2)
z = s(1);
z_d = s(2);
theta = s(3);
theta_d = s(4);

M_term = (M+m)-m*(cos(beta+theta))^2;
z_ddot = (1/M_term)*(m*g*sin(theta)*cos(beta+theta)+m*L*(theta_d)^2*sin(beta+theta)-K*z+(M+m)*g*sin(beta));

sdot = [z_d
    z_ddot
    theta_d
    (1/L)*(-g*sin(theta)-z_ddot*cos(beta+theta))];
end