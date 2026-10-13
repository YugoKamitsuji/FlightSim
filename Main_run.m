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
% load('./wind/U.mat')
% global U

%% Dryden model horizontal wind
% % sample noise time appears has to be set as one of the parameters of the
% % Dryden wind model. If the step size of the solver is changed, it is
% % cruicial to update it!
% % we use a predefined mean wind speed
% load('wind/wind_table.mat');
% % wind magnitude will be set through a interpolation block in:
% Wind_dir = 180 + 60;                      % deg clockwise from north
% Angle_wind_at6m = 180 + 90 - atan(1.728/2.932)*180/pi ;
% %Wind_dir = 30 + 180 ;
% %Angle_wind_at6m = 180 + atan(1.728/2.932)*180/pi ;
% V_wind_at6m = sqrt(2.932^2+1.728^2);      % m/s

%% Quadcopter physical parameters -- prsshin - hexacopter -- reduce the mass for a quad-compatibility

% dynamics related
m=1.035 * 4/6;                      % mass [kg]
l=0.225 ;                           % arm length [m]
Ix= 0.0469 ;
Iy= 0.0358 ;
Iz=  0.101 * 4/6;
I_B = diag([Ix,Iy,Iz]);
I_r = 3.357e-5;                     % rotor momemt of inertia (kg x m^2)

% aerodynamics related
b =  1.5652e-08 * (60/(2*pi))^2 ;   % Thrust coeffcient intially N/(rpm^2), now N/(rev/s^2)
k =  2.0862e-10 * (60/(2*pi))^2 ;   % Torque Coeffcient intially Nm/(rpm^2), now Nm/(rev/s^2)
R=3*0.0254;                         % propeller radius  [m]
nb=2;                               % number of blade
A=pi*R^2;                           % disk area
% sig=nb*mean(c)/(pi*R);              % solidity

% blade characteristics
rho = 1.15;
nr = 20;                             % number of radial points on a blade
npsi = 60;                            % number of azimuthal points for 2*Pi

%the0=repmat(5,[1,11]);
%cla1=repmat(1.8059*pi,[1,11]);
the0=repmat(4,[1,11]);
cla1=repmat(1.7059*pi,[1,11]);
th1=(the0+[24.9844849214694,24.4885730384207,23.6542985258914,22.4816610238794,20.9706612541160,19.1212982695717,16.9335723826041,14.4074838117246,11.5430326339323,8.34021849685304,4.79904143902087])*pi/180;
c1=[7.96284784614477,11.2448599794330,13.6346682267195,15.1322722373498,15.7376722802413,15.4508682842241,14.2718602574625,12.2006482106621,9.23723208187088,5.38161196932697,0.633787695366624]*0.001;

% interpolating data for different radius location (final size of the vectors 1*nr)
th=interp1(linspace(1/11,1,11),th1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');
c=interp1(linspace(1/11,1,11),c1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');
cla=interp1(linspace(1/11,1,11),cla1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');

r=linspace(floor(nr*0.01)/nr,1,nr);    % normolized radial locations
psi=linspace(0,2*pi,npsi);             % azimuth angle

maxsize=max(nr,npsi);
numvar=11;
geometry2=zeros(numvar,maxsize);

list={R,nb,A,rho,nr,npsi,th,c,cla,r,psi};

for  i=1:numvar
    
geometry2(i,1:length(list{i}))=[list{i}];

end

geometry = Simulink.Signal;
geometry .DataType = 'double';
geometry .Dimensions = [length(list) npsi];
geometry .Complexity = 'real';
geometry .SamplingMode = 'Sample based';
geometry .InitialValue = 'geometry2';

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

%% Considered mission parameters -- ascend - cruise - descend
% phase 1
tf_phase1 = 10; %sec
% zf_phase1 = -60; % for circle
zf_phase1 = -40; % for circle

% phase2
length_phase2 = 1; %min
tf_phase2 = tf_phase1 + length_phase2*60;
xdot_cruise = 15; %m/s

%phase3
t_phase3 = 10; %sec
tsim = tf_phase2 + t_phase3;
deltaT = 15; % (sec) transition time from hover to cruise and cruise to hover

%% Control parameters
% Aosition control

Kp = diag([1,1,1]);
Kd = diag([2,2,2]);
Ki = diag([0.01,0.01,0.01]);
% Attitude control
Tbar = diag([15,15,15]);
Gama1 = diag([3,3,3]); 
Gama2 = diag([0.1,0.1,0.1]);
Lambda = diag([10,10,10]);
%    break

%% Run simulink file
sim('M_6DoF_BS_Full_backup_11_25_2018.slx')

%%
reset(gcf);reset(gca)
set(0,'defaultLineLineWidth',2)
set(0,'defaultAxesFontSize', 18)
set(0,'defaultTextFontSize', 18)

T_t = Pos.time;
Position = Pos.signals.values;

figure(1)
figure(1)
clf
ax = axes;
hold(ax, 'on')

% 3D表示用のデータ
xNom = Position(:,1);
yNom = Position(:,2);
zNom = -Position(:,3);

xAct = Position(:,4);
yAct = Position(:,5);
zAct = -Position(:,6);

% 直方体サイズ [x y z]
boxSize = [200 200 50];
boxColor = [0.7 0.7 0.7];

% 軌道の中心を基準に、両脇に配置
xCenter = mean([min(xNom) max(xNom)]);
yCenter = mean([min(yNom) max(yNom)]);
zCenter = boxSize(3) / 2;     % 地面(z=0)から高さ50mの箱

yOffset = boxSize(2) / 2 + 40; % 軌道から少し離す

% 表示範囲
xMin = min([xNom; xAct; xCenter - boxSize(1)/2]);
xMax = max([xNom; xAct; xCenter + boxSize(1)/2]);
yMin = min([yNom; yAct; yCenter - yOffset - boxSize(2)/2]);
yMax = max([yNom; yAct; yCenter + yOffset + boxSize(2)/2]);
zMax = max([zNom; zAct; boxSize(3)]);

% z = 0 の基準面
[Xp, Yp] = meshgrid(linspace(xMin, xMax, 2), linspace(yMin, yMax, 2));
surf(Xp, Yp, zeros(size(Xp)), ...
    'FaceAlpha', 0.08, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5], ...
    'HandleVisibility', 'off');

% 直方体を2つ描画
drawCuboid3D(ax, [xCenter, yCenter - yOffset, zCenter], boxSize, boxColor);
drawCuboid3D(ax, [xCenter, yCenter + yOffset, zCenter], boxSize, boxColor);

% 目標軌道
plot3(xNom, yNom, zNom, '--r', ...
    'LineWidth', 1.8, 'DisplayName', 'Nominal trajectory')

% 時刻ラベル
T_t = Pos.time;
dx = 0.02 * (xMax - xMin);
dy = 0.02 * (yMax - yMin);
dz = 0.02 * zMax;

text(xAct(1) + dx, yAct(1) + dy, zAct(1) + dz, ...
    't = 0 s', 'Color', 'k', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom')
text(xAct(end) + dx, yAct(end) + dy, zAct(end) + dz, ...
    sprintf('t = %.1f s', T_t(end)), 'Color', 'k', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom')
text(xAct(end) + dx, yAct(end) - dy, zAct(end) - dz, ...
    sprintf('z = %.2f m', zAct(end)), ...
    'Color', 'k', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'top')

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
grid on
box on
view(3)

ax.FontSize = 18;
ax.LineWidth = 1.2;

colormap(parula)
caxis([T_t(1) T_t(end)])

legend('Location', 'best', 'FontSize', 20)

xlim([xMin xMax])
ylim([yMin yMax])
zlim([0 zMax + 20])



figure(2)
subplot(3,1,1)
plot(T_t,Position(:,1)/1000,'--b',T_t,Position(:,4)/1000,'r')
legend('Nominal','Actucal')
set(gca,'xticklabel',{[]})
ylabel('x (km)')
subplot(3,1,2)
plot(T_t,Position(:,2),'--b',T_t,Position(:,5),'r')
set(gca,'xticklabel',{[]})
ylabel('y (m)')
subplot(3,1,3)
plot(T_t,Position(:,3),'--b',T_t,Position(:,6),'r')
xlabel('Time (sec)')
ylabel('z (m)')

Velocities = Vel.signals.values;
figure(3)
subplot(3,1,1)
plot(T_t,Velocities(:,1),'--b',T_t,Velocities(:,4),'r')
ylabel('v_x (m/s)')
legend('Nominal','Actucal')
set(gca,'xticklabel',{[]})
subplot(3,1,2)
plot(T_t,Velocities(:,2),'--b',T_t,Velocities(:,5),'r')
ylabel('v_y (m/s)')
set(gca,'xticklabel',{[]})
subplot(3,1,3)
plot(T_t,Velocities(:,3),'--b',T_t,Velocities(:,6),'r')
ylabel('v_z (m/s)')
xlabel('Time (sec)')


figure(4)
subplot(3,1,1)
plot(Pos.time,Angles)
set(gca,'xticklabel',{[]})
ylabel('angles (deg)')
legend('\psi','\theta','\phi')
subplot(3,1,2)
plot(T_t,Omega)
set(gca,'xticklabel',{[]})
ylabel('\Omega (rad/s)')
legend('p','q','r')
subplot(3,1,3)
plot(T_t,Torques)
xlabel('Time (sec)')
ylabel('\tau (N m)')
legend('\tau_x','\tau_y','\tau_z')


figure(5)
plot(T_t,Rpms_fixed*60/(2*pi));hold on
plot(T_t,Rpms*60/(2*pi))
xlabel('Time (sec)')
ylabel('Propeller speed (rad/s)')
legend('\omega_1','\omega_2','\omega_3','\omega_4')

figure(6)
plot(T_t,Wind)
xlabel('Time (sec)')
ylabel('Wind speed (m/s)')
legend('w_x','w_y','w_z')

figure(7)
plot(T_t,Wind(:,3)./sqrt(sum(Wind(:,1:2).*Wind(:,1:2),2)))
xlabel('Time (sec)')
ylabel('$\frac{V_z}{\sqrt{v_x^2+v_y^2}}$','Interpreter', 'latex')

function drawCuboid3D(ax, center, sizeVec, faceColor)
dx = sizeVec(1) / 2;
dy = sizeVec(2) / 2;
dz = sizeVec(3) / 2;

x = center(1);
y = center(2);
z = center(3);

V = [x-dx y-dy z-dz;
    x+dx y-dy z-dz;
    x+dx y+dy z-dz;
    x-dx y+dy z-dz;
    x-dx y-dy z+dz;
    x+dx y-dy z+dz;
    x+dx y+dy z+dz;
    x-dx y+dy z+dz];

F = [1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8];

patch(ax, 'Vertices', V, 'Faces', F, ...
    'FaceColor', faceColor, 'EdgeColor', [0.35 0.35 0.35], ...
    'FaceAlpha', 0.75, 'HandleVisibility', 'off');
end

