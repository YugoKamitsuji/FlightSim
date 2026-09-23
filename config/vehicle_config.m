% dynamics related parameters
m=1.035 * 4/6;                      % mass [kg]
l=0.225 ;                           % arm length [m]
Ix= 0.0469 ;
Iy= 0.0358 ;
Iz=  0.101 * 4/6;
I_B = diag([Ix,Iy,Iz]);
I_r = 3.357e-5;                     % rotor momemt of inertia (kg x m^2)
