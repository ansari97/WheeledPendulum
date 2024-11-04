function K = controllerGainLQR(wheel_param)

% [l, m, I_r, r, M, I_w] = wheel_param;

% gravitational acceleration;
g = 9.81;

l= wheel_param(1);
m= wheel_param(2);
I_r= wheel_param(3);
r= wheel_param(4);
M= wheel_param(5);
I_w= wheel_param(6);

A = [0, 0, 1, 0;
    0, 0, 0, 1;
    0, 0, 0, 0;
    0, 0, 0, 0;];

A(3, 2) = m^2*r^2*l^2*g/(M*l^2*m*r^2 + I_r*M*r^2 + I_r*m*r^2 + I_w*l^2*m + I_r*I_w);

A(4, 2) = (M*r^2 + m*r^2 + I_w)*m*g*l/(M*l^2*m*r^2 + I_r*M*r^2 + I_r*m*r^2 + I_w*l^2*m + I_r*I_w);

B = zeros(4,1);
B(3, 1) = -(l^2*m + I_r)*r/(M*l^2*m*r^2 + I_r*M*r^2 + I_r*m*r^2 + I_w*l^2*m + I_r*I_w) - m*r^2*l/(M*l^2*m*r^2 + I_r*M*r^2 + I_r*m*r^2 + I_w*l^2*m + I_r*I_w);
B(4, 1) = -m*l*r/(M*l^2*m*r^2 + I_r*M*r^2 + I_r*m*r^2 + I_w*l^2*m + I_r*I_w) - (M*r^2 + m*r^2 + I_w)/(M*l^2*m*r^2 + I_r*M*r^2 + I_r*m*r^2 + I_w*l^2*m + I_r*I_w);

[V,D] = eig(ctrb(A, B));
rank((ctrb(A, B)));

%% Penalties on state and inputs
% State penalty
Q = [0.00001, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1;];

% Penalty on torque input
R = 1;

%% Solution to the LQR optimization
[K,S,CLP] = lqr(A,B,Q,R);

end