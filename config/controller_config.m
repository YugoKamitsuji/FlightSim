%% Control parameters
% Position control

Kp = diag([1,1,1]);
Kd = diag([2,2,2]);
Ki = diag([0.01,0.01,0.01]);
% Attitude control
Tbar = diag([15,15,15]);
Gama1 = diag([3,3,3]); 
Gama2 = diag([0.1,0.1,0.1]);
Lambda = diag([10,10,10]);
