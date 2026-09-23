function [R_d, V_d, psi, rates_ref, A_d] = ...
    ref_trajectory2(t, tf_1, zf_1, tf_2, t_phase3, xdot_cruise, deltaT)

% refTrajectory2
%
% Generate the reference trajectory for an array of time values.
%
% Input:
%   t             : time vector [1 x N] or [N x 1]
%   tf_1          : end time of the ascent phase
%   zf_1          : final altitude of the ascent phase
%   tf_2          : end time of the cruise phase
%   t_phase3      : duration of the descent phase
%   xdot_cruise   : cruise velocity in x direction
%   deltaT        : acceleration/deceleration duration
%
% Output:
%   R_d           : reference position [3 x N]
%   V_d           : reference velocity [3 x N]
%   psi           : reference yaw angle [1 x N]
%   rates_ref     : reference angular rates [3 x N]
%   A_d           : reference acceleration [3 x N]

%% Time vector
t = t(:).';     % Convert t to a row vector
N = length(t);

%% Initialization
x = zeros(1, N);
y = zeros(1, N);
z = zeros(1, N);

xdot = zeros(1, N);
ydot = zeros(1, N);
zdot = zeros(1, N);

xddot = zeros(1, N);
yddot = zeros(1, N);
zddot = zeros(1, N);





%% Reference attitude
psi = zeros(1, N);
% Original:
% psi = sin(2*pi*t/80);

rates_ref = zeros(3, N);

%% Output

R_d = [
    x;
    y;
    z
];

V_d = [
    xdot;
    ydot;
    zdot
];

A_d = [
    xddot;
    yddot;
    zddot
];

end