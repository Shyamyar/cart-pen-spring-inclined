clc;
clear;
close all;

%% Given Problem Parameters
g = 9.81; %gravitational acceleration (m/s^2)
K = 100; %spring stiffness (N/m)
L = 7; %bar length (m)
M = 50; %mass 1 (kg)
m = 20; %mass 2 (kg)
beta = deg2rad(30); %Inclined Angle, Constant in degrees

%% Initial Conditions
z0 = 0; z_d0 = 0;
theta0 = pi/2; theta_d0 = 0;
IC = [z0,z_d0,theta0,theta_d0];

%% Time Span
t0 = 0; tf = 8;
tspan = [t0,tf];
% tspan = t0:0.1:tf;
%% sdot = g(t,s)
sdot = @(t,s) inc_pen(t,s,K,L,M,m,beta);

%% Numerical Integration
[time, state_values] = ode45(sdot,tspan,IC);
z = state_values(:,1);
z_d = state_values(:,2);
theta = state_values(:,3);
theta_d = state_values(:,4);

%% Acceleration Calculation
M_term = (M+m).*ones(length(time),1)-m.*(cos(beta+theta)).^2;
z_dd = (1./M_term).*(m.*g.*sin(theta).*cos(beta+theta)+m.*L.*(theta_d).^2.*sin(beta+theta)-K.*z+(M+m).*g.*sin(beta).*ones(length(time),1));
theta_dd = (1/L).*(-g.*sin(theta)-z_dd.*cos(beta+theta));

%% Plot Results
figure(1), clf
plot(time,z), xlabel('time (s)'), ylabel('z (m)')
title('Mass(M) Z Displacement vs. Time')
grid on;

figure(2), clf
plot(time,z_d), xlabel('time (s)'), ylabel('z_d (m/s)')
title('Mass(M) Z Velocity vs. Time')
grid on;

figure(3), clf
plot(time,theta), xlabel('time (s)'), ylabel('\theta (rad)')
title('Pendulum(m) \theta Angular Displacement vs. Time')
grid on;

figure(4), clf
plot(time,theta_d), xlabel('time (s)'), ylabel('\theta_d (rad/s)')
title('Pendulum(m) \theta_d Angular Velocity vs. Time')
grid on;
%%
figure(5), clf
plot(time,z_dd), xlabel('time (s)'), ylabel('z_d_d (m/s^2)')
title('Mass(M) z_d_d Acceleration vs. Time')
grid on;

figure(6), clf
plot(time,theta_dd), xlabel('time (s)'), ylabel('\theta_d_d (rad/s^2)')
title('Pendulum(m) \theta_d_d Angular Acceleration vs. Time')
grid on;

%% Static Check
time_static = 53;
max_hyp = max(z(:,1));
drawincpend(z(time_static),theta(time_static),L,M,m,beta);

%% Visualizing the Motion
h=figure(7);
myVideo = VideoWriter('inc_pen_spring'); %open video file
myVideo.FrameRate = 5;  %can adjust this, 5 - 10 works well for me
open(myVideo)
h.Visible='on';
loops=length(time);
movie_(loops) = struct('cdata',[],'colormap',[]);
for k=1:loops
    drawincpend(z(k),theta(k),L,M,m,beta);
    movie_(k)=getframe;
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo);
movie(movie_);
