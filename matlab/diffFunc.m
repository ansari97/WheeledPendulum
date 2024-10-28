function qdot = diffFunc(t, q, param, T)
% diffFun is the system of differential equations describing the wheeled
% pendulum
%

% Unpack parameters
l = param(1);
m = param(2);
I_r = param(3);
r = param(4);
M = param(5);
I_w = param(6);

% gravitational acceleration;
g = 9.81;

A = I_w/r + r*(M + m);
B = m*r*l*cos(q(2));
C = m*l*cos(q(2));
D = I_r + m*l^2;

M = [-A, B;
    -C, D];

H = [1; 1];

f1 = m*r*l*sin(q(2)) * q(4)^2;
f2 = m*g*l*sin(q(2));

f = [f1; f2];

func = M\(H*T + f);

qdot(1) = q(3);
qdot(2) = q(4);
qdot(3) = func(1);
qdot(4) = func(2);

qdot = qdot';

end
