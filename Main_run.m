% This is the main run page of the quad-copter simulation
% written by Behdad Davoudi, Computational Aeroscience Lab 
% Aerospace Engineering Department, University of Michigan
% 2018

clearvars -except U
clc
dtr = pi/180;
%% Environment parameters
g = 9.81;   % gravitational acceleration (m/s^2)
cbar=0.04;  % drag coefficients

%% LES Wind this need to be done one time if you cleared the workspace already
%load('./wind/U.mat')
% global U
%% Dryden model horizontal wind
load('wind/wind_table.mat');
Wind_dir = 180 + 60;                      % deg clockwise from north
Angle_wind_at6m = 180 + 90 - atan(1.728/2.932)*180/pi;
V_wind_at6m = sqrt(2.932^2+1.728^2);      % m/s

%% parameters
run("config/vehicle_config.m");
run("config/aerodynamics_config.m");
run("config/controller_config.m");

%% Initial states
R_i = [0.0;0.0;0.0];
V_i = [0.0;0.0;-0.001];

% Initial orientation (Euler angles) w.r.t. the Earth inertial coordinate system
psi_i   = 0.0*dtr;
theta_i = 0.0*dtr;
phi_i   = 0.0*dtr;
InitialEulerAngles = [psi_i,theta_i,phi_i];

% initial body rates (rad/s)
Omega_i = [0;0;0];

%% Generate Trajectory
trajectory = generate_ref_trajectory();

% Plot reference trajectory contents
tPos  = trajectory.position.Time;
pos   = squeeze(trajectory.position.Data);
vel   = squeeze(trajectory.velocity.Data);
acc   = squeeze(trajectory.acceleration.Data);
yaw   = squeeze(trajectory.yaw.Data);
rates = squeeze(trajectory.rates.Data);

figure
subplot(3,2,1)
plot(tPos, pos)
grid on
ylabel('Position')
legend('x', 'y', 'z')

subplot(3,2,2)
plot(tPos, vel)
grid on
ylabel('Velocity')
legend('v_x', 'v_y', 'v_z')

subplot(3,2,3)
plot(tPos, acc)
grid on
ylabel('Acceleration')
legend('a_x', 'a_y', 'a_z')

subplot(3,2,4)
plot(tPos, yaw)
grid on
ylabel('Yaw')
legend('\psi')

subplot(3,2,5)
plot(tPos, rates)
grid on
ylabel('Rates')
xlabel('Time (s)')
legend('p', 'q', 'r')

figure
plot3(pos(:,1), pos(:,2), pos(:,3), 'b', 'LineWidth', 2)
grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('Reference trajectory')


%% Run simulink file
% sim('M_6DoF_BS_Full_backup_11_25_2018.slx')

%%
% reset(gcf);reset(gca)
% set(0,'defaultLineLineWidth',2)
% set(0,'defaultAxesFontSize',12)
% 
% T_t = Pos.time;
% Position = Pos.signals.values;
% figure(1)
% subplot(3,1,1)
% plot(T_t,Position(:,1)/1000,'--b',T_t,Position(:,4)/1000,'r')
% legend('Nominal','Actucal')
% set(gca,'xticklabel',{[]})
% ylabel('x (km)')
% subplot(3,1,2)
% plot(T_t,Position(:,2),'--b',T_t,Position(:,5),'r')
% set(gca,'xticklabel',{[]})
% ylabel('y (m)')
% subplot(3,1,3)
% plot(T_t,Position(:,3),'--b',T_t,Position(:,6),'r')
% xlabel('Time (sec)')
% ylabel('z (m)')
% 
% Velocities = Vel.signals.values;
% figure(2)
% subplot(3,1,1)
% plot(T_t,Velocities(:,1),'--b',T_t,Velocities(:,4),'r')
% legend('Nominal','Actucal')
% set(gca,'xticklabel',{[]})
% subplot(3,1,2)
% plot(T_t,Velocities(:,2),'--b',T_t,Velocities(:,5),'r')
% set(gca,'xticklabel',{[]})
% subplot(3,1,3)
% plot(T_t,Velocities(:,3),'--b',T_t,Velocities(:,6),'r')
% ylabel('v_z (m/s)')
% xlabel('Time (sec)')
% 
% 
% figure(3)
% subplot(3,1,1)
% plot(Pos.time,Angles)
% set(gca,'xticklabel',{[]})
% ylabel('angles (deg)')
% legend('\psi','\theta','\phi')
% subplot(3,1,2)
% plot(T_t,Omega)
% set(gca,'xticklabel',{[]})
% ylabel('\Omega (rad/s)')
% legend('p','q','r')
% subplot(3,1,3)
% plot(T_t,Torques)
% xlabel('Time (sec)')
% ylabel('\tau (N m)')
% legend('\tau_x','\tau_y','\tau_z')
% 
% 
% figure(4)
% plot(T_t,Rpms_fixed*60/(2*pi));hold on
% plot(T_t,Rpms*60/(2*pi))
% xlabel('Time (sec)')
% ylabel('Propeller speed (rad/s)')
% legend('\omega_1','\omega_2','\omega_3','\omega_4')
% 
% figure(5)
% plot(T_t,Wind)
% xlabel('Time (sec)')
% ylabel('Wind speed (m/s)')
% legend('w_x','w_y','w_z')
% 
% figure(6)
% plot3(Position(:,1),Position(:,2),-Position(:,3),'--b')
% hold on
% plot3(Position(:,4),Position(:,5),-Position(:,6),'r')
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% legend('Nominal trajectory','Actucal trajectory')
% 
% figure(7)
% plot(T_t,Wind(:,3)./sqrt(sum(Wind(:,1:2).*Wind(:,1:2),2)))
% xlabel('Time (sec)')
% ylabel('$\frac{V_z}{\sqrt{v_x^2+v_y^2}}$','Interpreter', 'latex')
