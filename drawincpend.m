function drawincpend(state_z,state_theta,L,M,m,beta)
%% Variable Intialization
z = 10+state_z;
th = state_theta;

%% Triangular Block
tria_x = [0 0 (2*L)/tan(beta) 0];  % cart width
tria_y = [0 -2*L -2*L 0];
plot(tria_x,tria_y,'k','LineWidth',2);
hold on;
fill(tria_x,tria_y,'r')

%% M Block (Cart)
W = 1*sqrt(M);  % cart width
H = .5*sqrt(M); % cart height
M_x = [z*cos(beta) z*cos(beta)+H*sin(beta) z*cos(beta)+H*sin(beta)+W*cos(beta) z*cos(beta)+W*cos(beta)];
M_y = [-z*sin(beta) -z*sin(beta)+H*cos(beta) -z*sin(beta)+H*cos(beta)-W*sin(beta) -z*sin(beta)-W*sin(beta)];
plot(M_x,M_y,'k','LineWidth',2);
fill(M_x,M_y,'b')

%% Spring Plot
plot([(H/2)*sin(beta) z*cos(beta)+(H/2)*sin(beta)],[(H/2)*cos(beta) -z*sin(beta)+(H/2)*cos(beta)],'k','LineWidth',3)

%% Pendulum Rod
p_i_x = z*cos(beta)+(H/2)*sin(beta)+(W/2)*cos(beta);
p_f_x = z*cos(beta)+(H/2)*sin(beta)+(W/2)*cos(beta)+L*sin(th);
p_i_y = -z*sin(beta)+(H/2)*cos(beta)-(W/2)*sin(beta);
p_f_y = -z*sin(beta)+(H/2)*cos(beta)-(W/2)*sin(beta)-L*cos(th);
plot([p_i_x p_f_x],[p_i_y p_f_y],'g','LineWidth',2)

%% m Circle (Pendulum)
mr = .08*sqrt(m);  % mass radius
rectangle('Position',[p_f_x-mr/2,p_f_y-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1);

%% Graph parameters
axis([-10 35 -30 10]);
axis equal
drawnow, hold off