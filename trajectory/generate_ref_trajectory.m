function trajectory = generate_ref_trajectory()

dt = 0.01;
t = 0:dt:80;

% reference trajectory 1
tf_1 = 10;
zf_1 = -40;
tf_2 = 70;
t_phase3 = 10;
xdot_cruise = 15;
deltaT = 15;

[R_d, V_d, psi, rates_ref, A_d] = ref_trajectory2( ...
    t, tf_1, zf_1, tf_2, t_phase3, xdot_cruise, deltaT ...
);


trajectory.position = timeseries(R_d, t);
trajectory.velocity = timeseries(V_d, t);
trajectory.acceleration = timeseries(A_d, t);
trajectory.yaw = timeseries(psi, t);
trajectory.rates = timeseries(rates_ref, t);

end